package com.ntu.mdp.t7.feature.connection.components

import androidx.compose.foundation.background
import androidx.compose.foundation.border
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.heightIn
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.size
import androidx.compose.foundation.shape.CircleShape
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.text.style.TextOverflow
import androidx.compose.ui.tooling.preview.Preview
import androidx.compose.ui.unit.dp
import com.ntu.mdp.t7.core.model.Direction
import com.ntu.mdp.t7.core.model.DiscoveredDevice
import com.ntu.mdp.t7.core.model.LinkState
import com.ntu.mdp.t7.core.model.RobotPose
import com.ntu.mdp.t7.core.model.SignalQuality
import com.ntu.mdp.t7.feature.task1_arena.LinkAction
import com.ntu.mdp.t7.feature.task1_arena.LinkColor
import com.ntu.mdp.t7.feature.task1_arena.components.PillButton
import com.ntu.mdp.t7.feature.task1_arena.linkDisplayFor
import com.ntu.mdp.t7.ui.theme.AppTheme
import com.ntu.mdp.t7.ui.theme.AppType
import com.ntu.mdp.t7.ui.theme.Dimens
import com.ntu.mdp.t7.ui.theme.MyApplicationTheme

@Composable
fun LinkBar(
    link: LinkState,
    bluetoothEnabled: Boolean,
    savedDevice: DiscoveredDevice?,
    robot: RobotPose,
    onScanClick: () -> Unit,
    onQuickConnectClick: () -> Unit,
    onClearSavedDeviceClick: () -> Unit,
    onConnectClick: () -> Unit,
    onDisconnectClick: () -> Unit,
    onEnableBluetoothClick: () -> Unit,
    modifier: Modifier = Modifier,
) {
    val colors = AppTheme.colors
    val display = linkDisplayFor(link, bluetoothEnabled, robot)
    val dotColor = when (display.color) {
        LinkColor.OFF -> colors.linkIdle
        LinkColor.NO_CONNECTION -> colors.linkLost
        LinkColor.WEAK -> colors.linkWorking
        LinkColor.CONNECTED -> colors.linkOk
    }

    Row(
        modifier = modifier
            .heightIn(min = 72.dp)
            .background(MaterialTheme.colorScheme.surface, RoundedCornerShape(Dimens.RadiusMedium))
            .border(Dimens.BorderWidth, MaterialTheme.colorScheme.outline, RoundedCornerShape(Dimens.RadiusMedium))
            .padding(horizontal = Dimens.Space16),
        verticalAlignment = Alignment.CenterVertically,
        horizontalArrangement = Arrangement.spacedBy(Dimens.Space16),
    ) {
        Box(modifier = Modifier.size(12.dp).background(dotColor, CircleShape))

        Column(modifier = Modifier.weight(1f)) {
            Text(
                text = display.title,
                style = AppType.Subtitle,
                color = MaterialTheme.colorScheme.onSurface,
                maxLines = 2,
                overflow = TextOverflow.Ellipsis,
            )
            display.subtitle?.let {
                Text(text = it, style = AppType.Mono, color = MaterialTheme.colorScheme.onSurfaceVariant)
            }
        }

        display.actions.forEach { action ->
            val (label, onClick) = when (action) {
                LinkAction.SCAN -> "Scan" to onScanClick
                LinkAction.CONNECT -> "Connect" to onConnectClick
                LinkAction.DISCONNECT -> "Disconnect" to onDisconnectClick
                LinkAction.ENABLE_BLUETOOTH -> "Turn on Bluetooth" to onEnableBluetoothClick
            }
            if (action == LinkAction.SCAN && savedDevice != null && link is LinkState.Idle && bluetoothEnabled) {
                Row(horizontalArrangement = Arrangement.spacedBy(Dimens.Space4)) {
                    PillButton(label = "Quick Connect: ${savedDevice.name}", onClick = onQuickConnectClick)
                    PillButton(label = "✕", onClick = onClearSavedDeviceClick)
                }
            } else {
                PillButton(label = label, onClick = onClick)
            }
        }
    }
}
