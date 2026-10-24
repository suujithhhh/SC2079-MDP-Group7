package com.ntu.mdp.t7.feature.task1_arena

import android.bluetooth.BluetoothManager
import android.content.Context
import androidx.activity.compose.rememberLauncherForActivityResult
import androidx.activity.result.contract.ActivityResultContracts
import androidx.compose.foundation.background
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.BoxWithConstraints
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.WindowInsets
import androidx.compose.foundation.layout.fillMaxHeight
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.systemBars
import androidx.compose.foundation.layout.windowInsetsPadding
import androidx.compose.foundation.pager.HorizontalPager
import androidx.compose.foundation.pager.rememberPagerState
import androidx.compose.material3.PrimaryTabRow
import androidx.compose.material3.Tab
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.CompositionLocalProvider
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableLongStateOf
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.rememberCoroutineScope
import androidx.compose.runtime.saveable.rememberSaveable
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.platform.LocalContext
import androidx.compose.ui.platform.LocalDensity
import androidx.compose.ui.unit.Density
import androidx.compose.ui.unit.dp
import androidx.lifecycle.compose.collectAsStateWithLifecycle
import androidx.lifecycle.viewmodel.compose.viewModel
import com.ntu.mdp.t7.core.bluetooth.BluetoothDiscovery
import com.ntu.mdp.t7.core.bluetooth.BluetoothPermissions
import com.ntu.mdp.t7.core.bluetooth.ClassicBluetoothTransport
import com.ntu.mdp.t7.core.bluetooth.DiscoveryEvent
import com.ntu.mdp.t7.core.bluetooth.FakeTransport
import com.ntu.mdp.t7.core.bluetooth.rememberBluetoothEnabled
import com.ntu.mdp.t7.core.bluetooth.rememberEnableBluetoothLauncher
import com.ntu.mdp.t7.core.bluetooth.rememberRequestDiscoverableLauncher
import com.ntu.mdp.t7.core.data.repository.DeviceRepositoryImpl
import com.ntu.mdp.t7.core.data.repository.PresetRepositoryImpl
import com.ntu.mdp.t7.core.model.Direction
import com.ntu.mdp.t7.core.model.DiscoveredDevice
import com.ntu.mdp.t7.core.model.LinkState
import com.ntu.mdp.t7.core.model.Obstacle
import com.ntu.mdp.t7.core.protocol.InboundMessage
import com.ntu.mdp.t7.core.protocol.ProtocolParser
import com.ntu.mdp.t7.feature.connection.ConnectionUiState
import com.ntu.mdp.t7.feature.connection.ConnectionViewModel
import com.ntu.mdp.t7.feature.connection.components.formatElapsedTime
import com.ntu.mdp.t7.feature.connection.components.DevicePickerSheet
import com.ntu.mdp.t7.feature.task1_arena.components.ObstacleFaceDialog
import com.ntu.mdp.t7.feature.task1_arena.components.ObstacleKeypadDialog
import com.ntu.mdp.t7.feature.task2_pathing.Task2TabContent
import com.ntu.mdp.t7.feature.task2_pathing.Task2UiState
import com.ntu.mdp.t7.feature.task2_pathing.Task2ViewModel
import java.util.Locale
import kotlin.time.Duration.Companion.milliseconds
import kotlinx.coroutines.awaitCancellation
import kotlinx.coroutines.delay
import kotlinx.coroutines.launch

private val ReferenceScreenWidth = 800.dp

@Composable
fun ArenaControllerScreen(
    task1State: Task1UiState,
    task2State: Task2UiState,
    connectionState: ConnectionUiState,
    bluetoothEnabled: Boolean,
    elapsedMs: Long,
    isTimerRunning: Boolean,
    onScanClick: () -> Unit,
    onQuickConnectClick: () -> Unit,
    onClearSavedDeviceClick: () -> Unit,
    onDeviceSelected: (DiscoveredDevice) -> Unit,
    onDevicePickerDismiss: () -> Unit,
    onConnectClick: () -> Unit,
    onDisconnectClick: () -> Unit,
    onEnableBluetoothClick: () -> Unit,
    onObstacleClick: (Obstacle) -> Unit,
    onGridClick: (GridCell) -> Unit,
    onObstaclePlacedWithFace: (GridCell, Direction) -> Unit,
    onDeleteObstacleClick: (Int) -> Unit,
    onColumnLabelClick: (Int) -> Unit,
    onRowLabelClick: (Int) -> Unit,
    onEditToggleClick: () -> Unit,
    onKeypadDialogOpen: () -> Unit,
    onKeypadDialogDismiss: () -> Unit,
    onPlaceObstacles: (List<Obstacle>) -> Unit,
    onStartClick: () -> Unit,
    onClearGridClick: () -> Unit,
    onRobotDragEnd: (GridCell) -> Unit,
    onRobotTapped: () -> Unit,
    onRobotFaceSelected: (Direction) -> Unit,
    onFaceSelected: (Direction) -> Unit,
    onFaceDialogDismiss: () -> Unit,
    onFaceDialogConfirm: () -> Unit,
    onSavePreset: (Int) -> Unit,
    onLoadPreset: (Int) -> Unit,
    onClearPreset: (Int) -> Unit,
    onPagerLockToggle: () -> Unit,
    onTask2Distance1Change: (Int) -> Unit,
    onTask2Distance2Change: (Int) -> Unit,
    onTask2Obs2LengthChange: (Int) -> Unit,
    modifier: Modifier = Modifier,
    prepElapsedMs: Long = 0L,
    isPrepTimerRunning: Boolean = false,
) {
    BoxWithConstraints(
        modifier = modifier
            .fillMaxSize()
            .background(ArenaColors.PageBackground)
            .windowInsetsPadding(WindowInsets.systemBars),
    ) {
        val scale = computeUiScale(maxWidth.value, maxHeight.value, ReferenceScreenWidth.value, 1100f)
        val baseDensity = LocalDensity.current
        val scaledDensity = remember(baseDensity, scale) {
            Density(density = baseDensity.density * scale, fontScale = baseDensity.fontScale)
        }

        CompositionLocalProvider(LocalDensity provides scaledDensity) {
            ArenaControllerContent(
                task1State = task1State,
                task2State = task2State,
                connectionState = connectionState,
                bluetoothEnabled = bluetoothEnabled,
                elapsedMs = elapsedMs,
                isTimerRunning = isTimerRunning,
                prepElapsedMs = prepElapsedMs,
                isPrepTimerRunning = isPrepTimerRunning,
                onScanClick = onScanClick,
                onQuickConnectClick = onQuickConnectClick,
                onClearSavedDeviceClick = onClearSavedDeviceClick,
                onDeviceSelected = onDeviceSelected,
                onDevicePickerDismiss = onDevicePickerDismiss,
                onConnectClick = onConnectClick,
                onDisconnectClick = onDisconnectClick,
                onEnableBluetoothClick = onEnableBluetoothClick,
                onObstacleClick = onObstacleClick,
                onGridClick = onGridClick,
                onObstaclePlacedWithFace = onObstaclePlacedWithFace,
                onDeleteObstacleClick = onDeleteObstacleClick,
                onColumnLabelClick = onColumnLabelClick,
                onRowLabelClick = onRowLabelClick,
                onEditToggleClick = onEditToggleClick,
                onKeypadDialogOpen = onKeypadDialogOpen,
                onKeypadDialogDismiss = onKeypadDialogDismiss,
                onPlaceObstacles = onPlaceObstacles,
                onStartClick = onStartClick,
                onClearGridClick = onClearGridClick,
                onRobotDragEnd = onRobotDragEnd,
                onRobotTapped = onRobotTapped,
                onRobotFaceSelected = onRobotFaceSelected,
                onFaceSelected = onFaceSelected,
                onFaceDialogDismiss = onFaceDialogDismiss,
                onFaceDialogConfirm = onFaceDialogConfirm,
                onSavePreset = onSavePreset,
                onLoadPreset = onLoadPreset,
                onClearPreset = onClearPreset,
                onPagerLockToggle = onPagerLockToggle,
                onTask2Distance1Change = onTask2Distance1Change,
                onTask2Distance2Change = onTask2Distance2Change,
                onTask2Obs2LengthChange = onTask2Obs2LengthChange,
            )
        }
    }
}

@Composable
private fun ArenaControllerContent(
    task1State: Task1UiState,
    task2State: Task2UiState,
    connectionState: ConnectionUiState,
    bluetoothEnabled: Boolean,
    elapsedMs: Long,
    isTimerRunning: Boolean,
    prepElapsedMs: Long = 0L,
    isPrepTimerRunning: Boolean = false,
    onScanClick: () -> Unit,
    onQuickConnectClick: () -> Unit,
    onClearSavedDeviceClick: () -> Unit,
    onDeviceSelected: (DiscoveredDevice) -> Unit,
    onDevicePickerDismiss: () -> Unit,
    onConnectClick: () -> Unit,
    onDisconnectClick: () -> Unit,
    onEnableBluetoothClick: () -> Unit,
    onObstacleClick: (Obstacle) -> Unit,
    onGridClick: (GridCell) -> Unit,
    onObstaclePlacedWithFace: (GridCell, Direction) -> Unit,
    onDeleteObstacleClick: (Int) -> Unit,
    onColumnLabelClick: (Int) -> Unit,
    onRowLabelClick: (Int) -> Unit,
    onEditToggleClick: () -> Unit,
    onKeypadDialogOpen: () -> Unit,
    onKeypadDialogDismiss: () -> Unit,
    onPlaceObstacles: (List<Obstacle>) -> Unit,
    onStartClick: () -> Unit,
    onClearGridClick: () -> Unit,
    onRobotDragEnd: (GridCell) -> Unit,
    onRobotTapped: () -> Unit,
    onRobotFaceSelected: (Direction) -> Unit,
    onFaceSelected: (Direction) -> Unit,
    onFaceDialogDismiss: () -> Unit,
    onFaceDialogConfirm: () -> Unit,
    onSavePreset: (Int) -> Unit,
    onLoadPreset: (Int) -> Unit,
    onClearPreset: (Int) -> Unit,
    onPagerLockToggle: () -> Unit,
    onTask2Distance1Change: (Int) -> Unit,
    onTask2Distance2Change: (Int) -> Unit,
    onTask2Obs2LengthChange: (Int) -> Unit,
) {
    val pagerState = rememberPagerState(initialPage = 0) { 2 }
    val coroutineScope = rememberCoroutineScope()

    Column(
        modifier = Modifier
            .fillMaxWidth()
            .fillMaxHeight()
            .background(ArenaColors.PanelBackground)
            .padding(16.dp),
        verticalArrangement = Arrangement.spacedBy(12.dp),
    ) {
        if (!task1State.isPagerLocked) {
            Row(
                modifier = Modifier.fillMaxWidth(),
                verticalAlignment = Alignment.CenterVertically,
            ) {
                PrimaryTabRow(
                    selectedTabIndex = pagerState.currentPage,
                    modifier = Modifier.fillMaxWidth(),
                    containerColor = ArenaColors.PanelBackground,
                    contentColor = ArenaColors.TextPrimary,
                ) {
                    Tab(
                        selected = pagerState.currentPage == 0,
                        onClick = { coroutineScope.launch { pagerState.animateScrollToPage(0) } },
                        text = { Text("Task 1") },
                    )
                    Tab(
                        selected = pagerState.currentPage == 1,
                        onClick = { coroutineScope.launch { pagerState.animateScrollToPage(1) } },
                        text = { Text("Task 2") },
                    )
                }
            }
        }

        HorizontalPager(
            state = pagerState,
            userScrollEnabled = !task1State.isPagerLocked,
            modifier = Modifier.weight(1f).fillMaxWidth(),
        ) { page ->
            if (page == 0) {
                Task1TabContent(
                    task1State = task1State,
                    connectionState = connectionState,
                    bluetoothEnabled = bluetoothEnabled,
                    elapsedMs = elapsedMs,
                    isTimerRunning = isTimerRunning,
                    prepElapsedMs = prepElapsedMs,
                    isPrepTimerRunning = isPrepTimerRunning,
                    onScanClick = onScanClick,
                    onQuickConnectClick = onQuickConnectClick,
                    onClearSavedDeviceClick = onClearSavedDeviceClick,
                    onConnectClick = onConnectClick,
                    onDisconnectClick = onDisconnectClick,
                    onEnableBluetoothClick = onEnableBluetoothClick,
                    onObstacleClick = onObstacleClick,
                    onGridClick = onGridClick,
                    onObstaclePlacedWithFace = onObstaclePlacedWithFace,
                    onEditToggleClick = onEditToggleClick,
                    onKeypadDialogOpen = onKeypadDialogOpen,
                    onStartClick = onStartClick,
                    onClearGridClick = onClearGridClick,
                    onRobotDragEnd = onRobotDragEnd,
                    onRobotTapped = onRobotTapped,
                    onRobotFaceSelected = onRobotFaceSelected,
                    onColumnLabelClick = onColumnLabelClick,
                    onRowLabelClick = onRowLabelClick,
                    onSavePreset = onSavePreset,
                    onLoadPreset = onLoadPreset,
                    onClearPreset = onClearPreset,
                    onPagerLockToggle = onPagerLockToggle,
                )
            } else {
                Task2TabContent(
                    task2State = task2State,
                    connectionState = connectionState,
                    robot = task1State.robot,
                    bluetoothEnabled = bluetoothEnabled,
                    isStartSent = task1State.isStartSent,
                    isPagerLocked = task1State.isPagerLocked,
                    elapsedMs = elapsedMs,
                    isTimerRunning = isTimerRunning,
                    isReady = task1State.isReady,
                    isConnected = connectionState.link is LinkState.Connected,
                    onScanClick = onScanClick,
                    onQuickConnectClick = onQuickConnectClick,
                    onClearSavedDeviceClick = onClearSavedDeviceClick,
                    onConnectClick = onConnectClick,
                    onDisconnectClick = onDisconnectClick,
                    onEnableBluetoothClick = onEnableBluetoothClick,
                    onStartClick = onStartClick,
                    onClearGridClick = onClearGridClick,
                    onTask2Distance1Change = onTask2Distance1Change,
                    onTask2Distance2Change = onTask2Distance2Change,
                    onTask2Obs2LengthChange = onTask2Obs2LengthChange,
                    onPagerLockToggle = onPagerLockToggle,
                )
            }
        }

        if (connectionState.devicePicker.isVisible) {
            DevicePickerSheet(
                isScanning = connectionState.devicePicker.isScanning,
                devices = connectionState.devicePicker.devices,
                onDeviceSelected = onDeviceSelected,
                onDismiss = onDevicePickerDismiss,
            )
        }

        task1State.obstacleFaceDialogTarget?.let { targetId ->
            val existingObstacle = task1State.obstacles.find { it.id == targetId }
            ObstacleFaceDialog(
                obstacleId = targetId,
                x = existingObstacle?.x ?: 0,
                y = existingObstacle?.y ?: 0,
                currentFace = existingObstacle?.face ?: task1State.obstacleFaceDraft,
                isNewPlacement = task1State.isNewPlacement,
                onFaceSelected = onFaceSelected,
                onDelete = { onDeleteObstacleClick(targetId) },
                onConfirm = onFaceDialogConfirm,
                onDismiss = onFaceDialogDismiss,
            )
        }

        if (task1State.isKeypadDialogOpen) {
            ObstacleKeypadDialog(
                initialObstacles = task1State.obstacles,
                onDismiss = onKeypadDialogDismiss,
                onPlaceObstacles = onPlaceObstacles,
            )
        }
    }
}

@Composable
fun ArenaControllerRoute(modifier: Modifier = Modifier) {
    val context = LocalContext.current
    val coroutineScope = rememberCoroutineScope()
    val adapter = remember {
        (context.getSystemService(Context.BLUETOOTH_SERVICE) as? BluetoothManager)?.adapter
    }

    val deviceRepository = remember(context) { DeviceRepositoryImpl(context) }
    val presetRepository = remember(context) { PresetRepositoryImpl(context) }

    val connectionViewModel: ConnectionViewModel = viewModel {
        val transport = if ((adapter != null) && BluetoothPermissions.allGranted(context)) {
            ClassicBluetoothTransport(context, adapter)
        } else {
            FakeTransport()
        }
        ConnectionViewModel(transport, deviceRepository)
    }

    val task1ViewModel: Task1ViewModel = viewModel {
        Task1ViewModel(presetRepository)
    }

    val task2ViewModel: Task2ViewModel = viewModel()

    val connectionState by connectionViewModel.uiState.collectAsStateWithLifecycle()
    val task1State by task1ViewModel.uiState.collectAsStateWithLifecycle()
    val task2State by task2ViewModel.uiState.collectAsStateWithLifecycle()
    val scanNonce by connectionViewModel.scanNonce.collectAsStateWithLifecycle()
    val bluetoothEnabled by rememberBluetoothEnabled()
    val enableBluetooth = rememberEnableBluetoothLauncher(
        onPermissionDenied = connectionViewModel::onBluetoothPermissionDenied,
    )
    val requestDiscoverable = rememberRequestDiscoverableLauncher()

    var isTimerRunning by rememberSaveable { mutableStateOf(value = false) }
    var elapsedMs by rememberSaveable { mutableLongStateOf(0L) }

    var isPrepTimerRunning by rememberSaveable { mutableStateOf(value = false) }
    var prepElapsedMs by rememberSaveable { mutableLongStateOf(0L) }

    LaunchedEffect(isTimerRunning) {
        if (isTimerRunning) {
            val startTime = System.currentTimeMillis() - elapsedMs
            while (isTimerRunning) {
                elapsedMs = System.currentTimeMillis() - startTime
                delay(50.milliseconds)
            }
            if (elapsedMs > 0) {
                connectionViewModel.addStatusLog("Run completed in ${formatElapsedTime(elapsedMs)}")
            }
        }
    }

    LaunchedEffect(isPrepTimerRunning) {
        if (isPrepTimerRunning) {
            val startTime = System.currentTimeMillis() - prepElapsedMs
            while (isPrepTimerRunning) {
                prepElapsedMs = System.currentTimeMillis() - startTime
                delay(50.milliseconds)
            }
        }
    }

    // Route incoming protocol lines to task ViewModels
    LaunchedEffect(Unit) {
        connectionViewModel.incomingLines.collect { line ->
            when (val message = ProtocolParser.parse(line)) {
                is InboundMessage.RobotPose -> {
                    task1ViewModel.updateRobotPose(message.x, message.y, message.direction)
                    connectionViewModel.addStatusLog("Robot at %02d,%02d facing %s".format(message.x, message.y, message.direction.code))
                }
                is InboundMessage.RobotPoseDistance -> {
                    // Task 2 live distance tracking removed
                }
                is InboundMessage.TargetFound -> {
                    task1ViewModel.updateObstacleTarget(message.obstacleId, message.targetId, message.face)
                    task2ViewModel.updateTargetInfo(message.obstacleId, message.targetId, message.imageUrl)
                    connectionViewModel.addStatusLog("Target ${message.targetId} read on obstacle ${message.obstacleId}")
                }
                is InboundMessage.StatusMessage -> {
                    connectionViewModel.addStatusLog(message.text)
                    val lower = message.text.lowercase(Locale.ROOT)
                    if (lower.contains("ready")) {
                        task1ViewModel.setReady(ready = true)
                        if (isPrepTimerRunning) {
                            isPrepTimerRunning = false
                            connectionViewModel.addStatusLog("Prep completed in ${formatElapsedTime(prepElapsedMs)}")
                        }
                    }
                    if (lower.contains("done") || lower.contains("complete") || lower.contains("finish") || lower.contains("stop") || lower.contains("end")) {
                        isTimerRunning = false
                    }
                }
                InboundMessage.Ready -> {
                    task1ViewModel.setReady(ready = true)
                    isTimerRunning = false
                    if (isPrepTimerRunning) {
                        isPrepTimerRunning = false
                        connectionViewModel.addStatusLog("Prep completed in ${formatElapsedTime(prepElapsedMs)}")
                    }
                }
                else -> Unit
            }
        }
    }

    var pendingScanAfterPermission by remember { mutableStateOf(value = false) }
    val permissionLauncher = rememberLauncherForActivityResult(
        ActivityResultContracts.RequestMultiplePermissions(),
    ) { result ->
        if (result.values.all { it }) {
            if (adapter != null) {
                connectionViewModel.upgradeTransport(ClassicBluetoothTransport(context, adapter))
                requestDiscoverable()
            }
            if (pendingScanAfterPermission) {
                connectionViewModel.onScanClick()
            }
        } else {
            connectionViewModel.onBluetoothPermissionDenied()
        }
        pendingScanAfterPermission = false
    }

    var startupBluetoothPromptShown by rememberSaveable { mutableStateOf(value = false) }
    LaunchedEffect(adapter) {
        if ((adapter != null) && !startupBluetoothPromptShown) {
            startupBluetoothPromptShown = true
            if (!BluetoothPermissions.allGranted(context)) {
                permissionLauncher.launch(BluetoothPermissions.required())
            }
        }
    }

    LaunchedEffect(connectionState.link, scanNonce, adapter) {
        if ((connectionState.link == LinkState.Scanning) && (adapter != null) && BluetoothPermissions.allGranted(context)) {
            val discovery = BluetoothDiscovery(context, adapter)
            discovery.start { event ->
                when (event) {
                    is DiscoveryEvent.DeviceFound ->
                        connectionViewModel.onDeviceFound(event.name, event.address, event.rssi, event.isPaired)
                    DiscoveryEvent.Finished -> connectionViewModel.onScanFinished()
                }
            }
            try {
                awaitCancellation()
            } finally {
                discovery.stop()
            }
        }
    }

    ArenaControllerScreen(
        task1State = task1State,
        task2State = task2State,
        connectionState = connectionState,
        bluetoothEnabled = bluetoothEnabled,
        elapsedMs = elapsedMs,
        isTimerRunning = isTimerRunning,
        prepElapsedMs = prepElapsedMs,
        isPrepTimerRunning = isPrepTimerRunning,
        onScanClick = {
            if (BluetoothPermissions.allGranted(context)) {
                requestDiscoverable()
                @Suppress("MissingPermission")
                val paired = adapter?.bondedDevices?.map {
                    DiscoveredDevice(name = it.name ?: "Unknown Device", address = it.address, isPaired = true)
                } ?: emptyList()
                connectionViewModel.onScanClick(paired)
            } else {
                pendingScanAfterPermission = true
                permissionLauncher.launch(BluetoothPermissions.required())
            }
        },
        onQuickConnectClick = connectionViewModel::onQuickConnectClick,
        onClearSavedDeviceClick = connectionViewModel::onClearSavedDeviceClick,
        onDeviceSelected = connectionViewModel::onDeviceSelected,
        onDevicePickerDismiss = connectionViewModel::onDevicePickerDismiss,
        onConnectClick = connectionViewModel::onConnectClick,
        onDisconnectClick = connectionViewModel::onDisconnectClick,
        onEnableBluetoothClick = enableBluetooth,
        onObstacleClick = task1ViewModel::onObstacleClick,
        onGridClick = { cell -> task1ViewModel.onGridClick(cell.x, cell.y) },
        onObstaclePlacedWithFace = { cell, face -> task1ViewModel.onObstaclePlacedWithFace(cell.x, cell.y, face) },
        onDeleteObstacleClick = task1ViewModel::onDeleteObstacleClick,
        onColumnLabelClick = task1ViewModel::onColumnLabelClick,
        onRowLabelClick = task1ViewModel::onRowLabelClick,
        onEditToggleClick = {
            task1ViewModel.onEditToggleClick { message ->
                connectionViewModel.logOutbound(message)
                connectionViewModel.addStatusLog("Sent plan to robot")
                connectionViewModel.addStatusLog("Grid submit time: ${formatElapsedTime(prepElapsedMs)}")
                coroutineScope.launch { connectionViewModel.transport.send(message) }
            }
        },
        onKeypadDialogOpen = {
            prepElapsedMs = 0L
            isPrepTimerRunning = true
            task1ViewModel.onKeypadDialogOpen()
        },
        onKeypadDialogDismiss = task1ViewModel::onKeypadDialogDismiss,
        onPlaceObstacles = task1ViewModel::onPlaceObstacles,
        onStartClick = {
            elapsedMs = 0L
            isTimerRunning = true
            task1ViewModel.onStartClick { message ->
                connectionViewModel.logOutbound(message)
                connectionViewModel.addStatusLog("Sent START signal")
                coroutineScope.launch { connectionViewModel.transport.send(message) }
            }
        },
        onClearGridClick = {
            isTimerRunning = false
            isPrepTimerRunning = false
            elapsedMs = 0L
            prepElapsedMs = 0L
            task1ViewModel.onClearGridClick()
            task2ViewModel.resetTask2Data()
        },
        onRobotDragEnd = { cell -> task1ViewModel.onRobotDragEnd(cell.x, cell.y) },
        onRobotTapped = task1ViewModel::onRobotTapped,
        onRobotFaceSelected = task1ViewModel::onRobotFaceSelected,
        onFaceSelected = task1ViewModel::onFaceSelected,
        onFaceDialogDismiss = task1ViewModel::onFaceDialogDismiss,
        onFaceDialogConfirm = task1ViewModel::onFaceDialogConfirm,
        onSavePreset = task1ViewModel::onSavePreset,
        onLoadPreset = task1ViewModel::onLoadPreset,
        onClearPreset = task1ViewModel::onClearPreset,
        onPagerLockToggle = task1ViewModel::onPagerLockToggle,
        onTask2Distance1Change = task2ViewModel::onTask2Distance1Change,
        onTask2Distance2Change = task2ViewModel::onTask2Distance2Change,
        onTask2Obs2LengthChange = task2ViewModel::onTask2Obs2LengthChange,
        modifier = modifier,
    )
}
