package com.ntu.mdp.t7.feature.task1_arena

import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.fillMaxHeight
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.width
import androidx.compose.runtime.Composable
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clipToBounds
import androidx.compose.ui.unit.dp
import com.ntu.mdp.t7.core.model.Direction
import com.ntu.mdp.t7.core.model.DiscoveredDevice
import com.ntu.mdp.t7.core.model.EditMode
import com.ntu.mdp.t7.core.model.Obstacle
import com.ntu.mdp.t7.feature.connection.ConnectionUiState
import com.ntu.mdp.t7.feature.connection.components.LinkBar
import com.ntu.mdp.t7.feature.connection.components.ProtocolLogDrawer
import com.ntu.mdp.t7.feature.task1_arena.components.ArenaGrid
import com.ntu.mdp.t7.feature.task1_arena.components.ObstacleCreationBar
import com.ntu.mdp.t7.feature.task1_arena.components.PresetPanel
import com.ntu.mdp.t7.feature.task1_arena.components.StatusPanel
import com.ntu.mdp.t7.feature.task1_arena.components.ZoomableBox

@Composable
fun Task1TabContent(
    task1State: Task1UiState,
    connectionState: ConnectionUiState,
    bluetoothEnabled: Boolean,
    elapsedMs: Long,
    isTimerRunning: Boolean,
    prepElapsedMs: Long = 0L,
    isPrepTimerRunning: Boolean = false,
    onScanClick: () -> Unit,
    onQuickConnectClick: () -> Unit,
    onClearSavedDeviceClick: () -> Unit,
    onConnectClick: () -> Unit,
    onDisconnectClick: () -> Unit,
    onEnableBluetoothClick: () -> Unit,
    onObstacleClick: (Obstacle) -> Unit,
    onGridClick: (GridCell) -> Unit,
    onObstaclePlacedWithFace: (GridCell, Direction) -> Unit,
    onEditToggleClick: () -> Unit,
    onKeypadDialogOpen: () -> Unit,
    onStartClick: () -> Unit,
    onClearGridClick: () -> Unit,
    onRobotDragEnd: (GridCell) -> Unit,
    onRobotTapped: () -> Unit,
    onRobotFaceSelected: (Direction) -> Unit,
    onColumnLabelClick: (Int) -> Unit,
    onRowLabelClick: (Int) -> Unit,
    onSavePreset: (Int) -> Unit,
    onLoadPreset: (Int) -> Unit,
    onClearPreset: (Int) -> Unit,
    onPagerLockToggle: () -> Unit,
    modifier: Modifier = Modifier,
) {
    Column(verticalArrangement = Arrangement.spacedBy(12.dp), modifier = modifier.fillMaxSize()) {
        ArenaGrid(
            obstacles = task1State.obstacles,
            robot = task1State.robot,
            editing = task1State.editMode == EditMode.EDITING,
            robotDirectionPickerVisible = task1State.robotDirectionPickerVisible,
            highlightedColumn = task1State.highlightedColumn,
            highlightedRow = task1State.highlightedRow,
            onObstacleClick = onObstacleClick,
            onGridClick = onGridClick,
            onObstaclePlacedWithFace = onObstaclePlacedWithFace,
            onRobotDragEnd = onRobotDragEnd,
            onRobotTapped = onRobotTapped,
            onRobotFaceSelected = onRobotFaceSelected,
            onColumnLabelClick = onColumnLabelClick,
            onRowLabelClick = onRowLabelClick,
            modifier = Modifier.fillMaxWidth()
        )

        Row(
            modifier = Modifier.fillMaxWidth(),
            horizontalArrangement = Arrangement.spacedBy(12.dp),
        ) {
            LinkBar(
                link = connectionState.link,
                bluetoothEnabled = bluetoothEnabled,
                savedDevice = connectionState.savedDevice,
                robot = task1State.robot,
                onScanClick = onScanClick,
                onQuickConnectClick = onQuickConnectClick,
                onClearSavedDeviceClick = onClearSavedDeviceClick,
                onConnectClick = onConnectClick,
                onDisconnectClick = onDisconnectClick,
                onEnableBluetoothClick = onEnableBluetoothClick,
                modifier = Modifier.weight(0.6f),
            )
            ObstacleCreationBar(
                editMode = task1State.editMode,
                isPlanSent = task1State.isPlanSent,
                isReady = task1State.isReady,
                isStartSent = task1State.isStartSent,
                onToggleClick = onEditToggleClick,
                onKeypadClick = onKeypadDialogOpen,
                onStartClick = onStartClick,
                onClearGridClick = onClearGridClick,
                modifier = Modifier.weight(0.4f),
            )
        }

        Row(
            modifier = Modifier.weight(1f).fillMaxWidth(),
            horizontalArrangement = Arrangement.spacedBy(12.dp),
        ) {
            PresetPanel(
                presets = task1State.presets,
                currentObstacles = task1State.obstacles,
                editMode = task1State.editMode,
                isPagerLocked = task1State.isPagerLocked,
                elapsedMs = elapsedMs,
                isTimerRunning = isTimerRunning,
                prepElapsedMs = prepElapsedMs,
                isPrepTimerRunning = isPrepTimerRunning,
                onPagerLockToggle = onPagerLockToggle,
                onSavePreset = onSavePreset,
                onLoadPreset = onLoadPreset,
                onClearPreset = onClearPreset,
                modifier = Modifier.width(300.dp).fillMaxHeight(),
            )
            Column(
                modifier = Modifier.weight(1f).fillMaxHeight(),
                verticalArrangement = Arrangement.spacedBy(12.dp),
            ) {
                StatusPanel(
                    entries = connectionState.statusLog,
                    modifier = Modifier.weight(1f).fillMaxWidth(),
                )
                ProtocolLogDrawer(
                    messages = connectionState.rawProtocolLog,
                    modifier = Modifier.fillMaxWidth(),
                )
            }
        }
    }
}
