package com.example.myapplication.arena

/** Demo data reproducing the shipped mockup, for previews and the no-transport demo route. */
fun sampleArenaUiState(): ArenaUiState = ArenaUiState(
    link = LinkState.Connected(deviceName = "HC-05", address = "98:D3:31:FB:2A:C4", signal = SignalQuality.STRONG),
    robot = RobotPose(x = 7, y = 4, direction = Direction.NORTH),
    obstacles = listOf(
        Obstacle(id = 1, x = 2, y = 16),
        Obstacle(id = 2, x = 5, y = 12),
        Obstacle(id = 3, x = 15, y = 17),
        Obstacle(id = 4, x = 9, y = 9, face = Direction.EAST),
        Obstacle(id = 5, x = 13, y = 6, targetId = 4, face = Direction.WEST),
        Obstacle(id = 6, x = 17, y = 3, isSelected = true),
    ),
    statusLog = listOf(
        StatusMessage("14:02:11", "Connected to HC-05", StatusLevel.INFO),
        StatusMessage("14:02:38", "Obstacle 5 placed at 13,06", StatusLevel.INFO),
        StatusMessage("14:03:04", "Target 4 read on obstacle 5, face W", StatusLevel.EVENT),
        StatusMessage("14:03:21", "Move refused. Obstacle 6 blocks the path.", StatusLevel.ALERT),
        StatusMessage("14:03:40", "Robot at 07,04 facing N", StatusLevel.EVENT),
    ),
)

/** Fake device list for the picker sheet demo — no real Bluetooth scan exists yet. */
fun sampleDiscoveredDevices(): List<DiscoveredDevice> = listOf(
    DiscoveredDevice(name = "HC-05", address = "98:D3:31:FB:2A:C4", signalStrength = -42),
    DiscoveredDevice(name = "AMD-Tool-07", address = "34:14:B5:8A:11:9C", signalStrength = -61),
    DiscoveredDevice(name = "Nexus 7", address = "40:4E:36:9C:2F:01", signalStrength = -78),
)
