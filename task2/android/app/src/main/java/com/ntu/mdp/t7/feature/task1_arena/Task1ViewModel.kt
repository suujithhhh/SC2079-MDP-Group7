package com.ntu.mdp.t7.feature.task1_arena

import androidx.lifecycle.ViewModel
import androidx.lifecycle.viewModelScope
import com.ntu.mdp.t7.core.data.repository.PresetRepository
import com.ntu.mdp.t7.core.model.ArenaConfig
import com.ntu.mdp.t7.core.model.Direction
import com.ntu.mdp.t7.core.model.EditMode
import com.ntu.mdp.t7.core.model.Obstacle
import com.ntu.mdp.t7.core.model.ObstaclePreset
import com.ntu.mdp.t7.core.model.RobotPose
import com.ntu.mdp.t7.core.model.nextObstacleId
import com.ntu.mdp.t7.core.model.obstaclesMissingFaces
import com.ntu.mdp.t7.core.protocol.ProtocolSerializer
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.flow.update
import kotlinx.coroutines.launch

class Task1ViewModel(
    private val presetRepository: PresetRepository? = null
) : ViewModel() {

    private val _uiState = MutableStateFlow(Task1UiState())
    val uiState: StateFlow<Task1UiState> = _uiState.asStateFlow()

    init {
        viewModelScope.launch(Dispatchers.IO) {
            loadPresets()
        }
    }

    private fun loadPresets() {
        val loaded = presetRepository?.loadPresets()
        if (loaded != null) {
            _uiState.update { it.copy(presets = loaded) }
        }
    }

    private fun Task1UiState.withObstacleDialogCleared(): Task1UiState = copy(
        obstacleFaceDialogTarget = null,
        isNewPlacement = false,
        obstacleIdDraft = "",
        obstacleFaceDraft = null,
        obstacleIdError = null,
    )

    fun onKeypadDialogOpen() {
        _uiState.update { it.copy(isKeypadDialogOpen = true) }
    }

    fun onKeypadDialogDismiss() {
        _uiState.update { it.copy(isKeypadDialogOpen = false) }
    }

    fun onPlaceObstacles(newObstacles: List<Obstacle>) {
        _uiState.update {
            it.copy(
                isKeypadDialogOpen = false,
                obstacles = newObstacles,
                editMode = EditMode.EDITING
            )
        }
    }

    fun onObstacleClick(clicked: Obstacle) {
        _uiState.update { state ->
            if (state.editMode == EditMode.EDITING) {
                state.withObstacleDialogCleared().copy(obstacleFaceDialogTarget = clicked.id, isNewPlacement = false)
            } else {
                state.copy(obstacles = state.obstacles.map { it.copy(isSelected = it.id == clicked.id && !it.isSelected) })
            }
        }
    }

    fun onEditToggleClick(onSendWireMessage: (String) -> Unit): Boolean {
        val current = _uiState.value
        when (current.editMode) {
            EditMode.EDITING -> {
                val faceless = obstaclesMissingFaces(current.obstacles)
                if (faceless.isNotEmpty()) {
                    return false
                }
                val message = ProtocolSerializer.planRequest(current.robot, current.obstacles)
                _uiState.update {
                    it.copy(
                        editMode = EditMode.LOCKED,
                        isPlanSent = true,
                        isReady = false,
                        robotDirectionPickerVisible = false,
                    )
                }
                onSendWireMessage(message)
                return true
            }
            EditMode.LOCKED -> {
                _uiState.update { it.copy(editMode = EditMode.EDITING, isPlanSent = false, isReady = false, isStartSent = false) }
                return true
            }
        }
    }

    fun onStartClick(onSendWireMessage: (String) -> Unit) {
        val message = ProtocolSerializer.start()
        _uiState.update {
            it.copy(
                isStartSent = true,
                isPagerLocked = true,
                isReady = false,
            )
        }
        onSendWireMessage(message)
    }

    fun onPagerLockToggle() {
        _uiState.update { it.copy(isPagerLocked = !it.isPagerLocked) }
    }

    private fun renumberObstacles(obstacles: List<Obstacle>): List<Obstacle> {
        return obstacles.sortedBy { it.id }.mapIndexed { index, obs ->
            obs.copy(id = index + 1)
        }
    }

    fun onGridClick(x: Int, y: Int) {
        _uiState.update { state ->
            if (state.editMode != EditMode.EDITING) return@update state

            val cellOccupiedByObstacle = state.obstacles.any { it.x == x && it.y == y }
            val footprint = robotFootprintCells(
                state.robot.x, state.robot.y, state.robot.direction,
                ArenaConfig.ROBOT_FOOTPRINT_LENGTH, ArenaConfig.ROBOT_FOOTPRINT_WIDTH,
            )
            if (cellOccupiedByObstacle || (x to y) in footprint) return@update state

            val newId = nextObstacleId(state.obstacles.map { it.id })
            val newObstacle = Obstacle(id = newId, x = x, y = y)

            state.copy(
                obstacles = state.obstacles + newObstacle,
                obstacleFaceDialogTarget = newId,
                isNewPlacement = true,
                obstacleIdDraft = newId.toString(),
                obstacleFaceDraft = null,
                obstacleIdError = null
            )
        }
    }

    fun onObstaclePlacedWithFace(x: Int, y: Int, face: Direction) {
        _uiState.update { state ->
            if (state.editMode != EditMode.EDITING) return@update state

            val cellOccupiedByObstacle = state.obstacles.any { it.x == x && it.y == y }
            val footprint = robotFootprintCells(
                state.robot.x, state.robot.y, state.robot.direction,
                ArenaConfig.ROBOT_FOOTPRINT_LENGTH, ArenaConfig.ROBOT_FOOTPRINT_WIDTH,
            )
            if (cellOccupiedByObstacle || (x to y) in footprint) return@update state

            val newId = nextObstacleId(state.obstacles.map { it.id })
            val newObstacle = Obstacle(id = newId, x = x, y = y, face = face)

            state.withObstacleDialogCleared().copy(
                obstacles = state.obstacles + newObstacle
            )
        }
    }

    fun onDeleteObstacleClick(id: Int) {
        _uiState.update { state ->
            val remaining = state.obstacles.filterNot { it.id == id }
            val renumbered = renumberObstacles(remaining)
            state.withObstacleDialogCleared().copy(obstacles = renumbered)
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
                state.copy(obstacles = state.obstacles.map { if (it.id == targetId) it.copy(face = face) else it })
            } else {
                state
            }
        }
    }

    fun onFaceDialogConfirm() {
        _uiState.update { state -> state.withObstacleDialogCleared() }
    }

    fun onFaceDialogDismiss() {
        _uiState.update { state ->
            if (state.isNewPlacement) {
                val targetId = state.obstacleFaceDialogTarget
                if (targetId != null) {
                    val remaining = state.obstacles.filterNot { it.id == targetId }
                    val renumbered = renumberObstacles(remaining)
                    return@update state.withObstacleDialogCleared().copy(obstacles = renumbered)
                }
            }
            state.withObstacleDialogCleared()
        }
    }

    fun onRobotDragEnd(x: Int, y: Int) {
        _uiState.update { state ->
            val gridSize = ArenaConfig.GRID_SIZE
            val (sizeX, sizeY) = footprintBox(
                state.robot.direction, ArenaConfig.ROBOT_FOOTPRINT_LENGTH, ArenaConfig.ROBOT_FOOTPRINT_WIDTH,
            )

            val bottomLeftX = when (state.robot.direction) {
                Direction.NORTH, Direction.SOUTH -> x - (ArenaConfig.ROBOT_FOOTPRINT_WIDTH / 2)
                Direction.EAST, Direction.WEST -> x
            }
            val bottomLeftY = when (state.robot.direction) {
                Direction.NORTH, Direction.SOUTH -> y
                Direction.EAST, Direction.WEST -> y - (ArenaConfig.ROBOT_FOOTPRINT_WIDTH / 2)
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

            val newFootprint = robotFootprintCells(
                originX, originY, state.robot.direction,
                ArenaConfig.ROBOT_FOOTPRINT_LENGTH, ArenaConfig.ROBOT_FOOTPRINT_WIDTH,
            )
            val occupied = state.obstacles.map { it.x to it.y }.toSet()
            if (newFootprint.any { it in occupied }) return@update state

            state.copy(
                robot = state.robot.copy(x = originX, y = originY),
                robotDirectionPickerVisible = true,
            )
        }
    }

    fun onRobotFaceSelected(direction: Direction) {
        _uiState.update { state ->
            state.copy(
                robot = state.robot.copy(direction = direction),
                robotDirectionPickerVisible = false,
            )
        }
    }

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
            )
        }
    }

    fun onSavePreset(slotIndex: Int) {
        val state = _uiState.value
        if (state.obstacles.isEmpty()) return

        val cleanObstacles = state.obstacles.map { it.copy(targetId = null, isSelected = false) }
        val preset = ObstaclePreset(slotIndex = slotIndex, obstacles = cleanObstacles)

        val newPresets = state.presets.toMutableList()
        if (slotIndex in 0 until 3) {
            newPresets[slotIndex] = preset
        }

        _uiState.update { it.copy(presets = newPresets) }
        presetRepository?.savePreset(preset)
    }

    fun onLoadPreset(slotIndex: Int) {
        _uiState.update { state ->
            if (state.editMode != EditMode.EDITING) return@update state
            val preset = state.presets.getOrNull(slotIndex) ?: return@update state

            state.withObstacleDialogCleared().copy(obstacles = preset.obstacles)
        }
    }

    fun onClearPreset(slotIndex: Int) {
        _uiState.update { state ->
            val newPresets = state.presets.toMutableList()
            if (slotIndex in 0 until 3) {
                newPresets[slotIndex] = null
            }
            state.copy(presets = newPresets)
        }
        presetRepository?.clearPreset(slotIndex)
    }

    fun updateRobotPose(x: Int, y: Int, direction: Direction) {
        _uiState.update { it.copy(robot = RobotPose(x, y, direction)) }
    }

    fun updateObstacleTarget(obstacleId: Int, targetId: Int, face: Direction?) {
        _uiState.update { state ->
            val updated = state.obstacles.map { obstacle ->
                if (obstacle.id == obstacleId) {
                    obstacle.copy(targetId = targetId, face = face ?: obstacle.face)
                } else obstacle
            }
            state.copy(obstacles = updated)
        }
    }

    fun setReady(ready: Boolean) {
        _uiState.update { it.copy(isReady = ready, isStartSent = if (ready) false else it.isStartSent) }
    }

    private fun footprintBox(direction: Direction, length: Int, width: Int): Pair<Int, Int> =
        when (direction) {
            Direction.NORTH, Direction.SOUTH -> width to length
            Direction.EAST, Direction.WEST -> length to width
        }

    private fun robotFootprintCells(originX: Int, originY: Int, direction: Direction, length: Int, width: Int): Set<Pair<Int, Int>> {
        val cells = mutableSetOf<Pair<Int, Int>>()
        val (sizeX, sizeY) = footprintBox(direction, length, width)
        for (dx in 0 until sizeX) {
            for (dy in 0 until sizeY) {
                cells.add((originX + dx) to (originY + dy))
            }
        }
        return cells
    }
}
