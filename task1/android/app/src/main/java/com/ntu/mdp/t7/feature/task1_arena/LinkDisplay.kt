package com.ntu.mdp.t7.feature.task1_arena

import com.ntu.mdp.t7.core.model.LinkState
import com.ntu.mdp.t7.core.model.RobotPose
import com.ntu.mdp.t7.core.model.SignalQuality

enum class LinkAction { SCAN, CONNECT, DISCONNECT, ENABLE_BLUETOOTH }

enum class LinkColor { OFF, NO_CONNECTION, WEAK, CONNECTED }

data class LinkDisplay(
    val color: LinkColor,
    val title: String,
    val subtitle: String?,
    val actions: List<LinkAction>,
)

fun linkDisplayFor(link: LinkState, bluetoothEnabled: Boolean, robot: RobotPose): LinkDisplay {
    if (!bluetoothEnabled) {
        return LinkDisplay(
            color = LinkColor.OFF,
            title = "Bluetooth is turned off",
            subtitle = null,
            actions = listOf(LinkAction.ENABLE_BLUETOOTH),
        )
    }
    return when (link) {
        LinkState.Idle ->
            LinkDisplay(LinkColor.NO_CONNECTION, "Not connected", null, listOf(LinkAction.SCAN))

        LinkState.Scanning ->
            LinkDisplay(LinkColor.NO_CONNECTION, "Scanning for devices…", null, emptyList())

        is LinkState.Connecting ->
            LinkDisplay(LinkColor.NO_CONNECTION, "Connecting to ${link.deviceName}…", null, emptyList())

        is LinkState.Connected -> {
            val weak = link.signal == SignalQuality.WEAK
            LinkDisplay(
                color = if (weak) LinkColor.WEAK else LinkColor.CONNECTED,
                title = if (weak) {
                    "Connected to ${link.deviceName} · ${link.address} (weak signal)"
                } else {
                    "Connected to ${link.deviceName} · ${link.address}"
                },
                subtitle = "x %02d y %02d · %s".format(robot.x, robot.y, robot.direction.code),
                actions = listOf(LinkAction.DISCONNECT),
            )
        }

        is LinkState.Lost -> LinkDisplay(
            color = LinkColor.NO_CONNECTION,
            title = "Connection lost. Reconnecting… · ${link.deviceName}",
            subtitle = null,
            actions = listOf(LinkAction.SCAN, LinkAction.CONNECT, LinkAction.DISCONNECT),
        )
    }
}
