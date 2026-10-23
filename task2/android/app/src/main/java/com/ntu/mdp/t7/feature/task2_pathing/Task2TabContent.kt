package com.ntu.mdp.t7.feature.task2_pathing

import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.fillMaxHeight
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.width
import androidx.compose.runtime.Composable
import androidx.compose.ui.Modifier
import androidx.compose.ui.unit.dp
import com.ntu.mdp.t7.core.model.RobotPose
import com.ntu.mdp.t7.feature.connection.ConnectionUiState
import com.ntu.mdp.t7.feature.connection.components.LinkBar
import com.ntu.mdp.t7.feature.connection.components.ProtocolLogDrawer
import com.ntu.mdp.t7.feature.task1_arena.components.StatusPanel
import com.ntu.mdp.t7.feature.task1_arena.components.ZoomableBox
import com.ntu.mdp.t7.feature.task2_pathing.components.Task2ControlPanel
import com.ntu.mdp.t7.feature.task2_pathing.components.Task2Diagram
import com.ntu.mdp.t7.feature.task2_pathing.components.Task2SettingsPanel

@Composable
fun Task2TabContent(
    task2State: Task2UiState,
    connectionState: ConnectionUiState,
    robot: RobotPose,
    bluetoothEnabled: Boolean,
    isStartSent: Boolean,
    isPagerLocked: Boolean,
    elapsedMs: Long,
    isTimerRunning: Boolean,
    onScanClick: () -> Unit,
    onQuickConnectClick: () -> Unit,
    onClearSavedDeviceClick: () -> Unit,
    onConnectClick: () -> Unit,
    onDisconnectClick: () -> Unit,
    onEnableBluetoothClick: () -> Unit,
    onStartClick: () -> Unit,
    onClearGridClick: () -> Unit,
    onTask2Distance1Change: (Int) -> Unit,
    onTask2Distance2Change: (Int) -> Unit,
    onTask2Obs2LengthChange: (Int) -> Unit,
    onPagerLockToggle: () -> Unit,
    modifier: Modifier = Modifier,
    isReady: Boolean = false,
    isConnected: Boolean = false,
) {
    Column(verticalArrangement = Arrangement.spacedBy(12.dp), modifier = modifier.fillMaxSize()) {
        Row(
            modifier = Modifier.weight(1f).fillMaxWidth(),
            horizontalArrangement = Arrangement.spacedBy(12.dp),
        ) {
            ZoomableBox(modifier = Modifier.weight(1f).fillMaxHeight()) {
                Task2Diagram(
                    dist1 = task2State.task2Distance1,
                    dist2 = task2State.task2Distance2,
                    obs2Length = task2State.task2Obs2Length,
                    obs1Arrow = task2State.task2Obs1Arrow,
                    obs2Arrow = task2State.task2Obs2Arrow,
                    obs1TargetId = task2State.task2Obs1TargetId,
                    obs2TargetId = task2State.task2Obs2TargetId,
                    obs1ImageUrl = task2State.task2Obs1ImageUrl,
                    obs2ImageUrl = task2State.task2Obs2ImageUrl,
                    robotStep = task2State.robotStep,
                    onClearClick = onClearGridClick,
                    modifier = Modifier.fillMaxSize(),
                )
            }
        }

        Row(modifier = Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.spacedBy(12.dp)) {
            LinkBar(
                link = connectionState.link,
                bluetoothEnabled = bluetoothEnabled,
                savedDevice = connectionState.savedDevice,
                robot = robot,
                onScanClick = onScanClick,
                onQuickConnectClick = onQuickConnectClick,
                onClearSavedDeviceClick = onClearSavedDeviceClick,
                onConnectClick = onConnectClick,
                onDisconnectClick = onDisconnectClick,
                onEnableBluetoothClick = onEnableBluetoothClick,
                modifier = Modifier.weight(0.6f),
            )
            Task2ControlPanel(
                onStartClick = onStartClick,
                isStartSent = isStartSent,
                isReady = isReady,
                isConnected = isConnected,
                modifier = Modifier.weight(0.4f),
            )
        }

        Row(
            modifier = Modifier.weight(1f).fillMaxWidth(),
            horizontalArrangement = Arrangement.spacedBy(12.dp),
        ) {
            Task2SettingsPanel(
                dist1 = task2State.task2Distance1,
                dist2 = task2State.task2Distance2,
                obs2Length = task2State.task2Obs2Length,
                onDist1Change = onTask2Distance1Change,
                onDist2Change = onTask2Distance2Change,
                onObs2LengthChange = onTask2Obs2LengthChange,
                isPagerLocked = isPagerLocked,
                elapsedMs = elapsedMs,
                isTimerRunning = isTimerRunning,
                onPagerLockToggle = onPagerLockToggle,
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
