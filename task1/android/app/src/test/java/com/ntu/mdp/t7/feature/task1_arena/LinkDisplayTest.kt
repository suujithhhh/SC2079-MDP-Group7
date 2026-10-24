package com.ntu.mdp.t7.feature.task1_arena

import com.ntu.mdp.t7.core.model.Direction
import com.ntu.mdp.t7.core.model.LinkState
import com.ntu.mdp.t7.core.model.RobotPose
import com.ntu.mdp.t7.core.model.SignalQuality
import org.junit.Assert.assertEquals
import org.junit.Test

class LinkDisplayTest {

    private val robot = RobotPose(x = 7, y = 4, direction = Direction.NORTH)

    @Test
    fun `bluetooth off overrides the link state`() {
        val display = linkDisplayFor(
            LinkState.Connected("HC-05", "98:D3:31:FB:2A:C4", SignalQuality.STRONG),
            bluetoothEnabled = false,
            robot,
        )
        assertEquals(LinkColor.OFF, display.color)
        assertEquals("Bluetooth is turned off", display.title)
        assertEquals(listOf(LinkAction.ENABLE_BLUETOOTH), display.actions)
    }

    @Test
    fun `idle shows not connected with a scan action`() {
        val display = linkDisplayFor(LinkState.Idle, bluetoothEnabled = true, robot)
        assertEquals(LinkColor.NO_CONNECTION, display.color)
        assertEquals("Not connected", display.title)
        assertEquals(listOf(LinkAction.SCAN), display.actions)
    }

    @Test
    fun `scanning shows no actions`() {
        val display = linkDisplayFor(LinkState.Scanning, bluetoothEnabled = true, robot)
        assertEquals(LinkColor.NO_CONNECTION, display.color)
        assertEquals("Scanning for devices…", display.title)
        assertEquals(emptyList<LinkAction>(), display.actions)
    }

    @Test
    fun `connecting shows no actions`() {
        val display = linkDisplayFor(
            LinkState.Connecting("HC-05", "98:D3:31:FB:2A:C4"),
            bluetoothEnabled = true,
            robot,
        )
        assertEquals(LinkColor.NO_CONNECTION, display.color)
        assertEquals("Connecting to HC-05…", display.title)
        assertEquals(emptyList<LinkAction>(), display.actions)
    }

    @Test
    fun `connected with strong signal is green with a disconnect action`() {
        val display = linkDisplayFor(
            LinkState.Connected("HC-05", "98:D3:31:FB:2A:C4", SignalQuality.STRONG),
            bluetoothEnabled = true,
            robot,
        )
        assertEquals(LinkColor.CONNECTED, display.color)
        assertEquals("Connected to HC-05 · 98:D3:31:FB:2A:C4", display.title)
        assertEquals("x 07 y 04 · N", display.subtitle)
        assertEquals(listOf(LinkAction.DISCONNECT), display.actions)
    }

    @Test
    fun `connected with weak signal is yellow and says so`() {
        val display = linkDisplayFor(
            LinkState.Connected("HC-05", "98:D3:31:FB:2A:C4", SignalQuality.WEAK),
            bluetoothEnabled = true,
            robot,
        )
        assertEquals(LinkColor.WEAK, display.color)
        assertEquals("Connected to HC-05 · 98:D3:31:FB:2A:C4 (weak signal)", display.title)
        assertEquals(listOf(LinkAction.DISCONNECT), display.actions)
    }

    @Test
    fun `lost shows reconnecting with scan, connect, and disconnect actions`() {
        val display = linkDisplayFor(
            LinkState.Lost("HC-05", "98:D3:31:FB:2A:C4"),
            bluetoothEnabled = true,
            robot,
        )
        assertEquals(LinkColor.NO_CONNECTION, display.color)
        assertEquals("Connection lost. Reconnecting… · HC-05", display.title)
        assertEquals(listOf(LinkAction.SCAN, LinkAction.CONNECT, LinkAction.DISCONNECT), display.actions)
    }
}
