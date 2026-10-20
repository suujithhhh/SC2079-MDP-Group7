package com.ntu.mdp.t7.feature.task1_arena

import com.ntu.mdp.t7.core.model.Direction
import com.ntu.mdp.t7.core.model.DiscoveredDevice
import com.ntu.mdp.t7.core.model.LinkState
import com.ntu.mdp.t7.core.model.Obstacle
import com.ntu.mdp.t7.core.model.RobotPose
import com.ntu.mdp.t7.core.model.SignalQuality
import com.ntu.mdp.t7.core.model.StatusLevel
import com.ntu.mdp.t7.core.model.StatusMessage

fun sampleDiscoveredDevices(): List<DiscoveredDevice> = listOf(
    DiscoveredDevice(name = "HC-05", address = "98:D3:31:FB:2A:C4", signalStrength = -42),
    DiscoveredDevice(name = "AMD-Tool-07", address = "34:14:B5:8A:11:9C", signalStrength = -61),
    DiscoveredDevice(name = "Nexus 7", address = "40:4E:36:9C:2F:01", signalStrength = -78),
)
