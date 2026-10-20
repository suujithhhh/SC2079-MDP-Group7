package com.example.myapplication.arena

import android.content.Context
import androidx.lifecycle.ViewModel
import androidx.lifecycle.viewModelScope
import com.example.myapplication.bluetooth.BluetoothTransport
import com.example.myapplication.bluetooth.ConnectionState
import com.example.myapplication.bluetooth.FakeTransport
import com.example.myapplication.protocol.InboundMessage
import com.example.myapplication.protocol.ProtocolParser
import com.example.myapplication.protocol.ProtocolSerializer
import java.text.SimpleDateFormat
import java.util.Date
import java.util.Locale
import kotlinx.coroutines.Job
import kotlinx.coroutines.delay
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.flow.update
import kotlinx.coroutines.launch

/** Number of automatic reconnect attempts after an unexpected disconnect before giving up. */
private const val MAX_RECONNECT_ATTEMPTS = 10

/** Delay between automatic reconnect attempts. */
private const val RECONNECT_DELAY_MS = 1500L

/**
 * Owns the connection to [transport] and turns protocol traffic into [ArenaUiState]/
 * [DevicePickerUiState] updates. See
 * the team design notes for the
 * bounded auto-reconnect loop, and the team design notes
 * for the original transport design.
 */
class ArenaViewModel(
    initialTransport: BluetoothTransport = FakeTransport(),
    startEditing: Boolean = false,
) : ViewModel() {

    private var transport: BluetoothTransport = initialTransport

    private val _uiState = MutableStateFlow<ArenaUiState>(
        defaultArenaUiState().let { if (startEditing) it.copy(editMode = EditMode.EDITING) else it },
    )
    val uiState: StateFlow<ArenaUiState> = _uiState.asStateFlow()

    private val _devicePicker = MutableStateFlow<DevicePickerUiState>(DevicePickerUiState())
    val devicePicker: StateFlow<DevicePickerUiState> = _devicePicker.asStateFlow()

    private val _scanNonce = MutableStateFlow<Int>(0)
    val scanNonce: StateFlow<Int> = _scanNonce.asStateFlow()

    private val timeFormat = SimpleDateFormat("HH:mm:ss", Locale.US)
    private var pendingConnectSignal: SignalQuality? = null
    private var collectorJob: Job? = null
    private var reconnectJob: Job? = null

    init {
        attachTransport(initialTransport)
    }

    /**
     * Subscribes to [t]'s connection-state/incoming-line flows and starts its accept loop,
     * cancelling any previous transport's collectors first. Safe to call again with a new
     * transport instance (see [upgradeTransport]).
     */
    private fun attachTransport(t: BluetoothTransport, context: Context? = null) {
        collectorJob?.cancel()
        collectorJob = viewModelScope.launch {
            launch { t.connectionState.collect { handleConnectionState(it, context) } }
            launch { t.incomingLines.collect(::handleIncomingLine) }
        }
        t.startAcceptLoop()
    }

    /**
     * Swaps in a real transport once Bluetooth permissions are actually granted (see
     * Critical #1 in the final review: a first-run [FakeTransport] must not persist forever).
     * No-ops if [newTransport] is already the current transport.
     */
    fun upgradeTransport(newTransport: BluetoothTransport, context: Context? = null) {
        if (transport === newTransport) return
        val old = transport
        transport = newTransport
        attachTransport(newTransport, context)
        old.shutdown()
    }

    override fun onCleared() {
        transport.shutdown()
    }

    // --- Scan / device picker ---

    fun onScanClick(pairedDevices: List<DiscoveredDevice> = emptyList()) {
        _devicePicker.value = DevicePickerUiState(isVisible = true, isScanning = true, devices = pairedDevices)
        _uiState.update { it.copy(link = LinkState.Scanning) }
        _scanNonce.update { it + 1 }
    }

    fun onDeviceFound(name: String, address: String, rssi: Int?, isPaired: Boolean = false) {
        _devicePicker.update { picker ->
            picker.copy(
                devices = picker.devices.filterNot { it.address == address } +
                    DiscoveredDevice(name = name, address = address, signalStrength = rssi, isPaired = isPaired),
            )
        }
    }

    fun onScanFinished() {
        _devicePicker.update { it.copy(isScanning = false) }
    }

    fun onDeviceSelected(device: DiscoveredDevice) {
        pendingConnectSignal = signalQualityFor(device.signalStrength)
        _devicePicker.value = DevicePickerUiState()
        connectDirectly(device.name, device.address)
    }

    fun onDevicePickerDismiss() {
        _devicePicker.value = DevicePickerUiState()
        _uiState.update { if (it.link == LinkState.Scanning) it.copy(link = LinkState.Idle) else it }
    }

    fun onConnectClick() {
        when (val link = _uiState.value.link) {
            is LinkState.Lost -> {
                reconnectJob?.cancel()
                connectDirectly(link.deviceName, link.address)
            }
            else -> Unit
        }
    }

    private fun connectDirectly(deviceName: String, address: String) {
        _uiState.update { it.copy(link = LinkState.Connecting(deviceName, address)) }
        viewModelScope.launch { transport.connectTo(address, deviceName) }
    }

    fun onDisconnectClick() {
        reconnectJob?.cancel()
        transport.disconnect()
        _uiState.update {
            it.copy(link = LinkState.Idle, isReady = false, statusLog = it.statusLog + logEntry("Disconnected"))
        }
    }

    fun onBluetoothPermissionDenied() {
        _uiState.update {
            it.copy(
                statusLog = it.statusLog + logEntry(
                    "Bluetooth permission denied. Enable it in Settings to use Bluetooth.",
                    StatusLevel.ALERT,
                ),
            )
        }
    }

    // --- Robot and Grid interaction ---

    private fun isConnected(): Boolean = _uiState.value.link is LinkState.Connected

    private fun sendCommand(line: String) {
        viewModelScope.launch { transport.send(line) }
    }

    // --- Obstacles (geometry/state logic moved verbatim from the old demo route) ---

    private fun ArenaUiState.withObstacleDialogCleared(): ArenaUiState = copy(
        obstacleFaceDialogTarget = null,
        isNewPlacement = false,
        obstacleIdDraft = "",
        obstacleFaceDraft = null,
        obstacleIdError = null,
    )

    fun onObstacleClick(clicked: Obstacle) {
        _uiState.update { state ->
            if (state.editMode == EditMode.EDITING) {
                state.withObstacleDialogCleared().copy(obstacleFaceDialogTarget = clicked.id, isNewPlacement = false)
            } else {
                state.copy(obstacles = state.obstacles.map { it.copy(isSelected = it.id == clicked.id && !it.isSelected) })
            }
        }
    }

    fun onEditToggleClick() {
        val current = _uiState.value
        when (current.editMode) {
            EditMode.EDITING -> {
                val faceless = obstaclesMissingFaces(current.obstacles)
                if (faceless.isNotEmpty()) {
                    _uiState.update {
                        it.copy(
                            statusLog = it.statusLog + logEntry(
                                "Submit refused. Set a face for obstacle(s): ${faceless.joinToString { o -> "${o.id}" }}.",
                                StatusLevel.ALERT,
                            ),
                        )
                    }
                    return
                }
                val message = ProtocolSerializer.planRequest(current.robot, current.obstacles)
                _uiState.update {
                    it.copy(
                        editMode = EditMode.LOCKED,
                        isPlanSent = true,
                        isReady = false,
                        robotDirectionPickerVisible = false,
                        rawProtocolLog = it.rawProtocolLog + "→ $message",
                        statusLog = it.statusLog + logEntry("Sent PLAN_REQUEST: ${current.obstacles.size} obstacle(s)"),
                    )
                }
                if (isConnected()) {
                    sendCommand(message)
                }
            }
            EditMode.LOCKED -> _uiState.update { it.copy(editMode = EditMode.EDITING, isPlanSent = false, isReady = false, isStartSent = false) }
        }
    }

    fun onStartClick() {
        val message = ProtocolSerializer.start()
        if (isConnected()) {
            _uiState.update {
                it.copy(
                    isStartSent = true,
                    isReady = false,
                    rawProtocolLog = it.rawProtocolLog + "→ $message",
                    statusLog = it.statusLog + logEntry("Sent START signal"),
                )
            }
            sendCommand(message)
        }
    }

    private fun renumberObstacles(obstacles: List<Obstacle>): List<Obstacle> {
        return obstacles.sortedBy { it.id }.mapIndexed { index, obs ->
            obs.copy(id = index + 1)
        }
    }

    fun onGridClick(cell: GridCell) {
        _uiState.update { state ->
            if (state.editMode != EditMode.EDITING) return@update state
            
            val cellOccupiedByObstacle = state.obstacles.any { it.x == cell.x && it.y == cell.y }
            val footprint = robotFootprintCells(
                state.robot.x, state.robot.y, state.robot.direction,
                ArenaConfig.ROBOT_FOOTPRINT_LENGTH, ArenaConfig.ROBOT_FOOTPRINT_WIDTH,
            )
            if (cellOccupiedByObstacle || cell in footprint) return@update state

            val newId = nextObstacleId(state.obstacles.map { it.id })
            val newObstacle = Obstacle(id = newId, x = cell.x, y = cell.y)
            
            state.copy(
                obstacles = state.obstacles + newObstacle,
                obstacleFaceDialogTarget = newId,
                isNewPlacement = true,
                obstacleIdDraft = newId.toString(),
                obstacleFaceDraft = null,
                obstacleIdError = null,
                statusLog = state.statusLog + logEntry("Obstacle $newId placed at (%02d,%02d)".format(cell.x, cell.y))
            )
        }
    }

    fun onObstaclePlacedWithFace(cell: GridCell, face: Direction) {
        _uiState.update { state ->
            if (state.editMode != EditMode.EDITING) return@update state

            val cellOccupiedByObstacle = state.obstacles.any { it.x == cell.x && it.y == cell.y }
            val footprint = robotFootprintCells(
                state.robot.x, state.robot.y, state.robot.direction,
                ArenaConfig.ROBOT_FOOTPRINT_LENGTH, ArenaConfig.ROBOT_FOOTPRINT_WIDTH,
            )
            if (cellOccupiedByObstacle || cell in footprint) return@update state

            val newId = nextObstacleId(state.obstacles.map { it.id })
            val newObstacle = Obstacle(id = newId, x = cell.x, y = cell.y, face = face)

            state.withObstacleDialogCleared().copy(
                obstacles = state.obstacles + newObstacle,
                statusLog = state.statusLog + logEntry("Obstacle $newId placed at (%02d,%02d) facing ${face.code}".format(cell.x, cell.y))
            )
        }
    }

    fun onDeleteObstacleClick(id: Int) {
        _uiState.update { state ->
            val remaining = state.obstacles.filterNot { it.id == id }
            val renumbered = renumberObstacles(remaining)
            state.withObstacleDialogCleared().copy(
                obstacles = renumbered,
                statusLog = state.statusLog + logEntry("Obstacle $id deleted"),
            )
        }
    }

    fun onColumnLabelClick(column: Int) {
        _uiState.update { state ->
            state.copy(highlightedColumn = if (state.highlightedColumn == column) null else column)
        }
    }

    fun onRowLabelClick(row: Int) {
        _uiState.update { state ->
            state.copy(highlightedRow = if (state.highlightedRow == row) null else row)
        }
    }

    fun onFaceSelected(face: Direction) {
        _uiState.update { state ->
            val targetId = state.obstacleFaceDialogTarget ?: return@update state
            val existing = state.obstacles.find { it.id == targetId }
            if (existing != null) {
                state.copy(
                    obstacles = state.obstacles.map { if (it.id == targetId) it.copy(face = face) else it },
                    statusLog = state.statusLog + logEntry("Obstacle $targetId face set to ${face.code}"),
                )
            } else {
                state
            }
        }
    }

    fun onFaceDialogConfirm() {
        _uiState.update { state ->
            state.withObstacleDialogCleared()
        }
    }

    fun onFaceDialogDismiss() {
        _uiState.update { state ->
            if (state.isNewPlacement) {
                val targetId = state.obstacleFaceDialogTarget
                if (targetId != null) {
                    val remaining = state.obstacles.filterNot { it.id == targetId }
                    val renumbered = renumberObstacles(remaining)
                    return@update state.withObstacleDialogCleared().copy(
                        obstacles = renumbered,
                        statusLog = state.statusLog + logEntry("Obstacle placement cancelled")
                    )
                }
            }
            state.withObstacleDialogCleared()
        }
    }

    /**
     * Moves the robot to [cell] while editing, mirroring obstacle drag. Unlike an obstacle,
     * the robot is never deleted: a drop outside the grid, or onto an occupied cell, is a
     * no-op and the marker snaps back to its pre-drag position. [cell] is the footprint's
     * bottom-left origin cell, already derived from the drag position by the caller
     * (see [com.example.myapplication.arena.originFromTopLeftCell]).
     */
    fun onRobotDragEnd(cell: GridCell) {
        _uiState.update { state ->
            val gridSize = ArenaConfig.GRID_SIZE
            val (sizeX, sizeY) = footprintBox(
                state.robot.direction, ArenaConfig.ROBOT_FOOTPRINT_LENGTH, ArenaConfig.ROBOT_FOOTPRINT_WIDTH,
            )
            
            val bottomLeftX = when (state.robot.direction) {
                Direction.NORTH, Direction.SOUTH -> cell.x - (ArenaConfig.ROBOT_FOOTPRINT_WIDTH / 2)
                Direction.EAST, Direction.WEST -> cell.x
            }
            val bottomLeftY = when (state.robot.direction) {
                Direction.NORTH, Direction.SOUTH -> cell.y
                Direction.EAST, Direction.WEST -> cell.y - (ArenaConfig.ROBOT_FOOTPRINT_WIDTH / 2)
            }
            
            if (bottomLeftX !in 0 until gridSize || bottomLeftY !in 0 until gridSize) return@update state

            val clampedBottomLeftX = bottomLeftX.coerceIn(0, gridSize - sizeX)
            val clampedBottomLeftY = bottomLeftY.coerceIn(0, gridSize - sizeY)
            
            val originX = when (state.robot.direction) {
                Direction.NORTH, Direction.SOUTH -> clampedBottomLeftX + (ArenaConfig.ROBOT_FOOTPRINT_WIDTH / 2)
                Direction.EAST, Direction.WEST -> clampedBottomLeftX
            }
            val originY = when (state.robot.direction) {
                Direction.NORTH, Direction.SOUTH -> clampedBottomLeftY
                Direction.EAST, Direction.WEST -> clampedBottomLeftY + (ArenaConfig.ROBOT_FOOTPRINT_WIDTH / 2)
            }
            
            val origin = GridCell(originX, originY)
            val newFootprint = robotFootprintCells(
                origin.x, origin.y, state.robot.direction,
                ArenaConfig.ROBOT_FOOTPRINT_LENGTH, ArenaConfig.ROBOT_FOOTPRINT_WIDTH,
            )
            val occupied = state.obstacles.map { GridCell(it.x, it.y) }.toSet()
            if (newFootprint.any { it in occupied }) return@update state

            state.copy(
                robot = state.robot.copy(x = origin.x, y = origin.y),
                robotDirectionPickerVisible = true,
                statusLog = state.statusLog + logEntry("Robot moved to (%02d,%02d)".format(origin.x, origin.y)),
            )
        }
    }

    /** Commits the direction picker's selection and closes it — see
     * the team design notes. */
    fun onRobotFaceSelected(direction: Direction) {
        _uiState.update { state ->
            state.copy(
                robot = state.robot.copy(direction = direction),
                robotDirectionPickerVisible = false,
                statusLog = state.statusLog + logEntry("Robot facing set to ${direction.code}"),
            )
        }
    }

    /** Toggles the direction picker's visibility — the operator can tap the robot's body at
     * any time (while editing) to open or close it, independent of dragging. See
     * the team design notes. */
    fun onRobotTapped() {
        _uiState.update { state ->
            state.copy(robotDirectionPickerVisible = !state.robotDirectionPickerVisible)
        }
    }

    fun onClearGridClick() {
        _uiState.update { state ->
            state.withObstacleDialogCleared().copy(
                obstacles = emptyList(),
                robot = RobotPose(x = 2, y = 1, direction = Direction.NORTH),
                robotDirectionPickerVisible = false,
                isPlanSent = false,
                isReady = false,
                isStartSent = false,
                statusLog = state.statusLog + logEntry("Grid reset"),
            )
        }
    }

    // --- Type obstacles (text entry) ---

    /** Opens the "Type obstacles" popup, pre-filled with what is already on the grid. */
    fun onTypeObstaclesClick() {
        _uiState.update { state ->
            if (state.editMode != EditMode.EDITING) return@update state
            state.withObstacleDialogCleared().copy(
                typeObstaclesVisible = true,
                typeObstaclesDraft = obstaclesToText(state.obstacles),
            )
        }
    }

    /**
     * Appends one keypad character: a digit, a comma, or a face letter (u/d/l/r); anything
     * else is ignored. A face letter ends the obstacle's line, so the next line is started
     * automatically (see [parseObstacleText]) and no new-line key is needed.
     */
    fun onTypeObstaclesChar(char: Char) {
        val c = char.lowercaseChar()
        val endsLine = c in OBSTACLE_TEXT_FACE_LETTERS
        if (!(c.isDigit() || c == ',' || endsLine)) return
        _uiState.update { state ->
            if (!state.typeObstaclesVisible) return@update state
            state.copy(typeObstaclesDraft = state.typeObstaclesDraft + c + (if (endsLine) "\n" else ""))
        }
    }

    /**
     * Deletes the last key press. Right after a face letter (which started a new line by
     * itself) it removes the letter and that line break together, so one tap undoes one key.
     */
    fun onTypeObstaclesBackspace() {
        _uiState.update { state ->
            if (!state.typeObstaclesVisible) return@update state
            val draft = state.typeObstaclesDraft
            state.copy(typeObstaclesDraft = draft.dropLast(if (draft.endsWith("\n")) 2 else 1))
        }
    }

    fun onTypeObstaclesClear() {
        _uiState.update { state ->
            if (!state.typeObstaclesVisible) return@update state
            state.copy(typeObstaclesDraft = "")
        }
    }

    fun onTypeObstaclesDismiss() {
        _uiState.update { it.copy(typeObstaclesVisible = false, typeObstaclesDraft = "") }
    }

    /**
     * Replaces every obstacle on the grid with the typed list (ids 1..n by line order) and
     * closes the popup. Does nothing unless every line parses (see [parseObstacleText]), so a
     * typo never leaves a half-placed grid. The robot is left where it is.
     */
    fun onTypeObstaclesPlace() {
        _uiState.update { state ->
            if (state.editMode != EditMode.EDITING || !state.typeObstaclesVisible) return@update state
            val parsed = parseObstacleText(
                text = state.typeObstaclesDraft,
                gridSize = ArenaConfig.GRID_SIZE,
                blockedCells = robotBlockedCells(state.robot),
            )
            if (!parsed.isPlaceable) return@update state
            val placed = parsed.obstacles
            val replaced = state.obstacles.size
            state.withObstacleDialogCleared().copy(
                obstacles = placed,
                typeObstaclesVisible = false,
                typeObstaclesDraft = "",
                statusLog = state.statusLog +
                    logEntry(
                        "Placed ${placed.size} obstacle(s) from text" + if (replaced > 0) " (replaced $replaced)" else "",
                        StatusLevel.EVENT,
                    ) +
                    placed.map { o ->
                        logEntry("Obstacle ${o.id} placed at (%02d,%02d) facing ${o.face?.code}".format(o.x, o.y))
                    },
            )
        }
    }

    // --- Quick Connect ---

    fun initSavedDevice(context: Context) {
        val saved = BluetoothStorage.loadLastDevice(context)
        _uiState.update { it.copy(savedDevice = saved) }
    }

    fun onQuickConnectClick() {
        val saved = _uiState.value.savedDevice ?: return
        connectDirectly(saved.name, saved.address)
    }

    fun onClearSavedDeviceClick(context: Context) {
        BluetoothStorage.clearLastDevice(context)
        _uiState.update { it.copy(savedDevice = null) }
    }

    // --- Transport plumbing ---

    private fun handleConnectionState(state: ConnectionState, context: Context? = null) {
        when (state) {
            is ConnectionState.Connected -> {
                reconnectJob?.cancel()
                reconnectJob = null
                context?.let { BluetoothStorage.saveLastDevice(it, state.deviceName, state.address) }
                _uiState.update { current ->
                    val signal = pendingConnectSignal ?: SignalQuality.WEAK
                    pendingConnectSignal = null
                    val newSavedDevice = DiscoveredDevice(name = state.deviceName, address = state.address, isPaired = true)
                    current.copy(
                        link = LinkState.Connected(state.deviceName, state.address, signal),
                        savedDevice = newSavedDevice,
                        statusLog = current.statusLog + logEntry("Connected to ${state.deviceName}"),
                    )
                }
            }
            ConnectionState.Disconnected, ConnectionState.Listening -> {
                // While a reconnect attempt is in flight, its own connectTo() calls also
                // pass through here as Disconnected on failure — the loop below owns those
                // transitions (Connecting -> Lost -> retry) directly; let it, rather than
                // this reducer's manual-connect-failure branch (Connecting -> Idle) racing
                // with it every attempt.
                if (reconnectJob?.isActive == true) return
                val previousLink = _uiState.value.link
                _uiState.update { current ->
                    when (val link = current.link) {
                        is LinkState.Connected -> current.copy(
                            link = LinkState.Lost(link.deviceName, link.address),
                            isReady = false,
                            statusLog = current.statusLog + logEntry(
                                "Connection lost. Reconnecting…",
                                StatusLevel.ALERT,
                            ),
                        )
                        is LinkState.Connecting -> current.copy(
                            link = LinkState.Idle,
                            isReady = false,
                            statusLog = current.statusLog + logEntry("Connection failed", StatusLevel.ALERT),
                        )
                        else -> current
                    }
                }
                if (previousLink is LinkState.Connected) {
                    startReconnectLoop(previousLink.deviceName, previousLink.address)
                }
            }
            ConnectionState.Connecting -> Unit // already reflected synchronously by onConnectClick
        }
    }

    /**
     * Retries [transport]'s `connectTo` up to [MAX_RECONNECT_ATTEMPTS] times,
     * [RECONNECT_DELAY_MS] apart, after an unexpected disconnect — connecting directly to
     * the last-known address (no re-scan; Classic RFCOMM connects by address regardless of
     * scan state). Reads [transport]'s own `connectionState.value` directly, not [uiState],
     * to decide whether an attempt succeeded, since that's updated synchronously by the
     * transport rather than depending on this class's own collector coroutine having run
     * yet. Cancelled the moment any connection succeeds (see [handleConnectionState]) or
     * the user acts manually ([onConnectClick], [onDisconnectClick]).
     */
    private fun startReconnectLoop(deviceName: String, address: String) {
        reconnectJob?.cancel()
        reconnectJob = viewModelScope.launch {
            repeat(MAX_RECONNECT_ATTEMPTS) {
                delay(RECONNECT_DELAY_MS)
                _uiState.update { it.copy(link = LinkState.Connecting(deviceName, address)) }
                transport.connectTo(address, deviceName)
                if (transport.connectionState.value is ConnectionState.Connected) return@launch
                _uiState.update { it.copy(link = LinkState.Lost(deviceName, address)) }
            }
            _uiState.update {
                it.copy(
                    statusLog = it.statusLog + logEntry(
                        "Could not reconnect after $MAX_RECONNECT_ATTEMPTS attempts — tap Connect to retry",
                        StatusLevel.ALERT,
                    ),
                )
            }
        }
    }

    private fun handleIncomingLine(rawLine: String) {
        _uiState.update { state ->
            val nextRawLog = state.rawProtocolLog + "← $rawLine"
            when (val message = ProtocolParser.parse(rawLine)) {
                is InboundMessage.RobotPose -> {
                    state.copy(
                        rawProtocolLog = nextRawLog,
                        robot = RobotPose(message.x, message.y, message.direction),
                        statusLog = state.statusLog + logEntry(
                            "Robot at %02d,%02d facing %s".format(message.x, message.y, message.direction.code),
                            StatusLevel.EVENT,
                        ),
                    )
                }
                is InboundMessage.TargetFound -> {
                    val obstacleExists = state.obstacles.any { it.id == message.obstacleId }
                    val updatedObstacles = if (obstacleExists) {
                        state.obstacles.map { obstacle ->
                            if (obstacle.id == message.obstacleId) {
                                obstacle.copy(targetId = message.targetId, face = message.face ?: obstacle.face)
                            } else {
                                obstacle
                            }
                        }
                    } else {
                        val indexMatch = state.obstacles.elementAtOrNull(message.obstacleId - 1)
                        if (indexMatch != null) {
                            state.obstacles.mapIndexed { index, obstacle ->
                                if (index + 1 == message.obstacleId) {
                                    obstacle.copy(targetId = message.targetId, face = message.face ?: obstacle.face)
                                } else {
                                    obstacle
                                }
                            }
                        } else if (state.obstacles.size == 1) {
                            state.obstacles.map { obstacle ->
                                obstacle.copy(targetId = message.targetId, face = message.face ?: obstacle.face)
                            }
                        } else {
                            state.obstacles
                        }
                    }
                    state.copy(
                        rawProtocolLog = nextRawLog,
                        obstacles = updatedObstacles,
                        statusLog = state.statusLog + logEntry(
                            "Target ${message.targetId} read on obstacle ${message.obstacleId}" +
                                (message.face?.let { ", face ${it.code}" } ?: ""),
                            StatusLevel.EVENT,
                        ),
                    )
                }
                is InboundMessage.TargetRejected -> {
                    state.copy(
                        rawProtocolLog = nextRawLog,
                        statusLog = state.statusLog + logEntry(
                            "Target ID ${message.targetId} is invalid",
                            StatusLevel.ALERT,
                        ),
                    )
                }
                is InboundMessage.StatusMessage -> {
                    state.copy(
                        rawProtocolLog = nextRawLog,
                        statusLog = state.statusLog + logEntry(message.text),
                    )
                }
                InboundMessage.Ready -> {
                    state.copy(
                        rawProtocolLog = nextRawLog,
                        isReady = true,
                        isStartSent = false,
                    )
                }
                null -> {
                    state.copy(
                        rawProtocolLog = nextRawLog,
                        statusLog = state.statusLog + logEntry("[Received] $rawLine"),
                    )
                }
            }
        }
    }

    private fun logEntry(text: String, level: StatusLevel = StatusLevel.INFO) =
        StatusMessage(time = timeFormat.format(Date()), message = text, level = level)
}
