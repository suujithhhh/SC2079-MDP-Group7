package com.example.myapplication.arena

/** Which action buttons LinkBar renders for a given display state, per
 * the team design notes. */
enum class LinkAction { SCAN, CONNECT, DISCONNECT, ENABLE_BLUETOOTH }

/** Semantic color bucket for the status dot. The UI layer maps these onto the existing
 * AppColors.link* tokens (linkIdle/linkLost/linkWorking/linkOk) — no new colors. */
enum class LinkColor { OFF, NO_CONNECTION, WEAK, CONNECTED }

data class LinkDisplay(
    val color: LinkColor,
    val title: String,
    val subtitle: String?,
    val actions: List<LinkAction>,
)

/**
 * Pure mapping from link state + the real Bluetooth-adapter-enabled flag to what LinkBar
 * shows. Bluetooth-off overrides the logical [link] state entirely.
 */
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
