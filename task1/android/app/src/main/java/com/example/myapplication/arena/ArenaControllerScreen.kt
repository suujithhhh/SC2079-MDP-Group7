package com.example.myapplication.arena

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
import androidx.compose.foundation.layout.width
import androidx.compose.foundation.layout.windowInsetsPadding
import androidx.compose.runtime.Composable
import androidx.compose.runtime.CompositionLocalProvider
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.saveable.rememberSaveable
import androidx.compose.runtime.setValue
import androidx.compose.ui.Modifier
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.geometry.Rect
import androidx.compose.ui.platform.LocalDensity
import androidx.compose.ui.tooling.preview.Preview
import androidx.compose.ui.unit.Density
import androidx.compose.ui.unit.dp
import com.example.myapplication.arena.components.ArenaGrid
import com.example.myapplication.arena.components.DevicePickerSheet
import com.example.myapplication.arena.components.LinkBar
import com.example.myapplication.arena.components.ObstacleCreationBar
import com.example.myapplication.arena.components.ObstacleFaceDialog
import com.example.myapplication.arena.components.ProtocolLogDrawer
import com.example.myapplication.arena.components.StatusPanel
import com.example.myapplication.arena.components.TypeObstaclesActions
import com.example.myapplication.arena.components.TypeObstaclesDialog
import android.bluetooth.BluetoothManager
import android.content.Context
import androidx.activity.compose.rememberLauncherForActivityResult
import androidx.activity.result.contract.ActivityResultContracts
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.ui.platform.LocalContext
import androidx.lifecycle.compose.collectAsStateWithLifecycle
import androidx.lifecycle.viewmodel.compose.viewModel
import com.example.myapplication.bluetooth.BluetoothDiscovery
import com.example.myapplication.bluetooth.BluetoothPermissions
import com.example.myapplication.bluetooth.ClassicBluetoothTransport
import com.example.myapplication.bluetooth.DiscoveryEvent
import com.example.myapplication.bluetooth.FakeTransport
import com.example.myapplication.bluetooth.rememberBluetoothEnabled
import com.example.myapplication.bluetooth.rememberEnableBluetoothLauncher
import com.example.myapplication.bluetooth.rememberRequestDiscoverableLauncher
import kotlinx.coroutines.awaitCancellation

/**
 * Reference width the mockup was designed at (Samsung Galaxy Tab A7 Lite, 8.7",
 * 800x1340px @ ~179ppi -> mdpi -> 800x1340dp), used only to compute [computeUiScale].
 * The screen itself flexes to the device's actual width.
 */
private val ReferenceScreenWidth = 800.dp

@Composable
fun ArenaControllerScreen(
    state: ArenaUiState,
    bluetoothEnabled: Boolean,
    devicePicker: DevicePickerUiState,
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
    onStartClick: () -> Unit,
    onClearGridClick: () -> Unit,
    onRobotDragEnd: (GridCell) -> Unit,
    onRobotTapped: () -> Unit,
    onRobotFaceSelected: (Direction) -> Unit,
    onFaceSelected: (Direction) -> Unit,
    onFaceDialogDismiss: () -> Unit,
    onFaceDialogConfirm: () -> Unit,
    modifier: Modifier = Modifier,
    typeObstacles: TypeObstaclesActions = TypeObstaclesActions(),
) {
    BoxWithConstraints(
        modifier = modifier
            .fillMaxSize()
            .background(ArenaColors.PageBackground)
            .windowInsetsPadding(WindowInsets.systemBars),
    ) {
        val scale = computeUiScale(maxWidth.value, ReferenceScreenWidth.value)
        val baseDensity = LocalDensity.current
        val scaledDensity = remember(baseDensity, scale) {
            Density(density = baseDensity.density * scale, fontScale = baseDensity.fontScale)
        }

        CompositionLocalProvider(LocalDensity provides scaledDensity) {
            ArenaControllerContent(
                state = state,
                bluetoothEnabled = bluetoothEnabled,
                devicePicker = devicePicker,
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
                onStartClick = onStartClick,
                onClearGridClick = onClearGridClick,
                onRobotDragEnd = onRobotDragEnd,
                onRobotTapped = onRobotTapped,
                onRobotFaceSelected = onRobotFaceSelected,
                onFaceSelected = onFaceSelected,
                onFaceDialogDismiss = onFaceDialogDismiss,
                onFaceDialogConfirm = onFaceDialogConfirm,
                typeObstacles = typeObstacles,
            )
        }
    }
}

@Composable
private fun ArenaControllerContent(
    state: ArenaUiState,
    bluetoothEnabled: Boolean,
    devicePicker: DevicePickerUiState,
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
    onStartClick: () -> Unit,
    onClearGridClick: () -> Unit,
    onRobotDragEnd: (GridCell) -> Unit,
    onRobotTapped: () -> Unit,
    onRobotFaceSelected: (Direction) -> Unit,
    onFaceSelected: (Direction) -> Unit,
    onFaceDialogDismiss: () -> Unit,
    onFaceDialogConfirm: () -> Unit,
    typeObstacles: TypeObstaclesActions,
) {
    var gridBoundsWindow by remember { mutableStateOf<Rect?>(null) }

    Column(
        modifier = Modifier
            .fillMaxWidth()
            .fillMaxHeight()
            .background(ArenaColors.PanelBackground)
            .padding(16.dp),
        verticalArrangement = Arrangement.spacedBy(12.dp),
    ) {
        ArenaGrid(
            obstacles = state.obstacles,
            robot = state.robot,
            editing = state.editMode == EditMode.EDITING,
            robotDirectionPickerVisible = state.robotDirectionPickerVisible,
            highlightedColumn = state.highlightedColumn,
            highlightedRow = state.highlightedRow,
            onObstacleClick = onObstacleClick,
            onGridClick = onGridClick,
            onObstaclePlacedWithFace = onObstaclePlacedWithFace,
            onRobotDragEnd = onRobotDragEnd,
            onRobotTapped = onRobotTapped,
            onRobotFaceSelected = onRobotFaceSelected,
            onGridBoundsChanged = { gridBoundsWindow = it },
            onColumnLabelClick = onColumnLabelClick,
            onRowLabelClick = onRowLabelClick
        )

        Row(
            modifier = Modifier.fillMaxWidth(),
            horizontalArrangement = Arrangement.spacedBy(12.dp),
        ) {
            LinkBar(
                link = state.link,
                bluetoothEnabled = bluetoothEnabled,
                savedDevice = state.savedDevice,
                robot = state.robot,
                onScanClick = onScanClick,
                onQuickConnectClick = onQuickConnectClick,
                onClearSavedDeviceClick = onClearSavedDeviceClick,
                onConnectClick = onConnectClick,
                onDisconnectClick = onDisconnectClick,
                onEnableBluetoothClick = onEnableBluetoothClick,
                modifier = Modifier.weight(0.6f),
            )
            ObstacleCreationBar(
                editMode = state.editMode,
                isPlanSent = state.isPlanSent,
                isReady = state.isReady,
                isStartSent = state.isStartSent,
                onToggleClick = onEditToggleClick,
                onStartClick = onStartClick,
                onClearGridClick = onClearGridClick,
                onTypeObstaclesClick = typeObstacles.onOpen,
                modifier = Modifier.weight(0.4f),
            )
        }

        Row(
            modifier = Modifier.weight(1f).fillMaxWidth(),
            horizontalArrangement = Arrangement.spacedBy(12.dp),
        ) {
            Column(
                modifier = Modifier.weight(1f).fillMaxHeight(),
                verticalArrangement = Arrangement.spacedBy(12.dp),
            ) {
                StatusPanel(
                    entries = state.statusLog,
                    modifier = Modifier.weight(1f).fillMaxWidth(),
                )
                ProtocolLogDrawer(
                    messages = state.rawProtocolLog,
                    modifier = Modifier.fillMaxWidth(),
                )
            }
        }

        if (devicePicker.isVisible) {
            DevicePickerSheet(
                isScanning = devicePicker.isScanning,
                devices = devicePicker.devices,
                onDeviceSelected = onDeviceSelected,
                onDismiss = onDevicePickerDismiss,
            )
        }

        state.obstacleFaceDialogTarget?.let { targetId ->
            val existingObstacle = state.obstacles.find { it.id == targetId }
            ObstacleFaceDialog(
                obstacleId = targetId,
                x = existingObstacle?.x ?: 0,
                y = existingObstacle?.y ?: 0,
                currentFace = existingObstacle?.face ?: state.obstacleFaceDraft,
                isNewPlacement = state.isNewPlacement,
                onFaceSelected = onFaceSelected,
                onDelete = { onDeleteObstacleClick(targetId) },
                onConfirm = onFaceDialogConfirm,
                onDismiss = onFaceDialogDismiss,
            )
        }

        if (state.typeObstaclesVisible) {
            TypeObstaclesDialog(
                draft = state.typeObstaclesDraft,
                robot = state.robot,
                actions = typeObstacles,
            )
        }
    }
}

/**
 * Real Bluetooth route: constructs an [ArenaViewModel] backed by [ClassicBluetoothTransport]
 * when a Bluetooth adapter and permissions are available, [FakeTransport] otherwise (always
 * the case in an emulator, which has no functioning Bluetooth radio). See
 * the team design notes.
 */
@Composable
fun ArenaControllerRoute(modifier: Modifier = Modifier) {
    val context = LocalContext.current
    val adapter = remember {
        (context.getSystemService(Context.BLUETOOTH_SERVICE) as? BluetoothManager)?.adapter
    }

    val viewModel: ArenaViewModel = viewModel {
        val transport = if (adapter != null && BluetoothPermissions.allGranted(context)) {
            ClassicBluetoothTransport(context, adapter)
        } else {
            FakeTransport()
        }
        ArenaViewModel(transport, startEditing = true)
    }
    val state by viewModel.uiState.collectAsStateWithLifecycle()
    val devicePicker by viewModel.devicePicker.collectAsStateWithLifecycle()
    val scanNonce by viewModel.scanNonce.collectAsStateWithLifecycle()
    val bluetoothEnabled by rememberBluetoothEnabled()
    val enableBluetooth = rememberEnableBluetoothLauncher(
        onPermissionDenied = viewModel::onBluetoothPermissionDenied,
    )
    val requestDiscoverable = rememberRequestDiscoverableLauncher()

    var pendingScanAfterPermission by remember { mutableStateOf(false) }
    val permissionLauncher = rememberLauncherForActivityResult(
        ActivityResultContracts.RequestMultiplePermissions(),
    ) { result ->
        if (result.values.all { it }) {
            if (adapter != null) {
                viewModel.upgradeTransport(ClassicBluetoothTransport(context, adapter))
                requestDiscoverable()
            }
            if (pendingScanAfterPermission) {
                viewModel.onScanClick()
            }
        } else {
            viewModel.onBluetoothPermissionDenied()
        }
        pendingScanAfterPermission = false
    }

    // Request Bluetooth permission and, once granted, the one-time "make visible" prompt
    // right when the screen first appears, instead of on every Scan click — see
    // the team design notes.
    // Gated by startupBluetoothPromptShown (rememberSaveable) so it fires once per process
    // lifetime, not once per composition — otherwise it re-prompts on every Activity
    // recreation (e.g. screen rotation).
    var startupBluetoothPromptShown by rememberSaveable { mutableStateOf(false) }
    LaunchedEffect(adapter) {
        viewModel.initSavedDevice(context)
        if (adapter != null && !startupBluetoothPromptShown) {
            startupBluetoothPromptShown = true
            if (BluetoothPermissions.allGranted(context)) {
                requestDiscoverable()
            } else {
                permissionLauncher.launch(BluetoothPermissions.required())
            }
        }
    }

    LaunchedEffect(state.link, scanNonce, adapter) {
        val currentAdapter = adapter
        if (state.link == LinkState.Scanning && currentAdapter != null && BluetoothPermissions.allGranted(context)) {
            val discovery = BluetoothDiscovery(context, currentAdapter)
            discovery.start { event ->
                when (event) {
                    is DiscoveryEvent.DeviceFound ->
                        viewModel.onDeviceFound(event.name, event.address, event.rssi, event.isPaired)
                    DiscoveryEvent.Finished -> viewModel.onScanFinished()
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
        state = state,
        bluetoothEnabled = bluetoothEnabled,
        devicePicker = devicePicker,
        onScanClick = {
            if (BluetoothPermissions.allGranted(context)) {
                @Suppress("MissingPermission")
                val paired = adapter?.bondedDevices?.map { 
                    DiscoveredDevice(name = it.name ?: "Unknown Device", address = it.address, isPaired = true) 
                } ?: emptyList()
                viewModel.onScanClick(paired)
            } else {
                pendingScanAfterPermission = true
                permissionLauncher.launch(BluetoothPermissions.required())
            }
        },
        onQuickConnectClick = viewModel::onQuickConnectClick,
        onClearSavedDeviceClick = { viewModel.onClearSavedDeviceClick(context) },
        onDeviceSelected = viewModel::onDeviceSelected,
        onDevicePickerDismiss = viewModel::onDevicePickerDismiss,
        onConnectClick = viewModel::onConnectClick,
        onDisconnectClick = viewModel::onDisconnectClick,
        onEnableBluetoothClick = enableBluetooth,
        onObstacleClick = viewModel::onObstacleClick,
        onGridClick = viewModel::onGridClick,
        onObstaclePlacedWithFace = viewModel::onObstaclePlacedWithFace,
        onDeleteObstacleClick = viewModel::onDeleteObstacleClick,
        onColumnLabelClick = viewModel::onColumnLabelClick,
        onRowLabelClick = viewModel::onRowLabelClick,
        onEditToggleClick = viewModel::onEditToggleClick,
        onStartClick = viewModel::onStartClick,
        onClearGridClick = viewModel::onClearGridClick,
        onRobotDragEnd = viewModel::onRobotDragEnd,
        onRobotTapped = viewModel::onRobotTapped,
        onRobotFaceSelected = viewModel::onRobotFaceSelected,
        onFaceSelected = viewModel::onFaceSelected,
        onFaceDialogDismiss = viewModel::onFaceDialogDismiss,
        onFaceDialogConfirm = viewModel::onFaceDialogConfirm,
        typeObstacles = TypeObstaclesActions(
            onOpen = viewModel::onTypeObstaclesClick,
            onChar = viewModel::onTypeObstaclesChar,
            onBackspace = viewModel::onTypeObstaclesBackspace,
            onClear = viewModel::onTypeObstaclesClear,
            onPlace = viewModel::onTypeObstaclesPlace,
            onDismiss = viewModel::onTypeObstaclesDismiss,
        ),
        modifier = modifier,
    )
}

@Preview(name = "Arena Controller - Portrait (Tab A7 Lite, 1.0x)", widthDp = 800, heightDp = 1340)
@Composable
private fun ArenaControllerScreenPreview() {
    ArenaControllerScreen(
        state = sampleArenaUiState(),
        bluetoothEnabled = true,
        devicePicker = DevicePickerUiState(),
        onScanClick = {},
        onQuickConnectClick = {},
        onClearSavedDeviceClick = {},
        onDeviceSelected = {},
        onDevicePickerDismiss = {},
        onConnectClick = {},
        onDisconnectClick = {},
        onEnableBluetoothClick = {},
        onObstacleClick = {},
        onGridClick = {},
        onObstaclePlacedWithFace = { _, _ -> },
        onDeleteObstacleClick = {},
        onColumnLabelClick = {},
        onRowLabelClick = {},
        onEditToggleClick = {},
        onStartClick = {},
        onClearGridClick = {},
        onRobotDragEnd = {},
        onRobotTapped = {},
        onRobotFaceSelected = {},
        onFaceSelected = {},
        onFaceDialogDismiss = {},
        onFaceDialogConfirm = {},
    )
}

@Preview(name = "Arena Controller - Portrait (narrow, 0.75x clamp)", widthDp = 600, heightDp = 1024)
@Composable
private fun ArenaControllerScreenNarrowPreview() {
    ArenaControllerScreen(
        state = sampleArenaUiState(),
        bluetoothEnabled = true,
        devicePicker = DevicePickerUiState(),
        onScanClick = {},
        onQuickConnectClick = {},
        onClearSavedDeviceClick = {},
        onDeviceSelected = {},
        onDevicePickerDismiss = {},
        onConnectClick = {},
        onDisconnectClick = {},
        onEnableBluetoothClick = {},
        onObstacleClick = {},
        onGridClick = {},
        onObstaclePlacedWithFace = { _, _ -> },
        onDeleteObstacleClick = {},
        onColumnLabelClick = {},
        onRowLabelClick = {},
        onEditToggleClick = {},
        onStartClick = {},
        onClearGridClick = {},
        onRobotDragEnd = {},
        onRobotTapped = {},
        onRobotFaceSelected = {},
        onFaceSelected = {},
        onFaceDialogDismiss = {},
        onFaceDialogConfirm = {},
    )
}

@Preview(name = "Arena Controller - Portrait (wide, 1.35x clamp)", widthDp = 1080, heightDp = 1620)
@Composable
private fun ArenaControllerScreenWidePreview() {
    ArenaControllerScreen(
        state = sampleArenaUiState(),
        bluetoothEnabled = true,
        devicePicker = DevicePickerUiState(),
        onScanClick = {},
        onQuickConnectClick = {},
        onClearSavedDeviceClick = {},
        onDeviceSelected = {},
        onDevicePickerDismiss = {},
        onConnectClick = {},
        onDisconnectClick = {},
        onEnableBluetoothClick = {},
        onObstacleClick = {},
        onGridClick = {},
        onObstaclePlacedWithFace = { _, _ -> },
        onDeleteObstacleClick = {},
        onColumnLabelClick = {},
        onRowLabelClick = {},
        onEditToggleClick = {},
        onStartClick = {},
        onClearGridClick = {},
        onRobotDragEnd = {},
        onRobotTapped = {},
        onRobotFaceSelected = {},
        onFaceSelected = {},
        onFaceDialogDismiss = {},
        onFaceDialogConfirm = {},
    )
}
