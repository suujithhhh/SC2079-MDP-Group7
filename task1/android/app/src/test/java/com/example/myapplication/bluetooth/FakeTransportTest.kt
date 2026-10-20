package com.example.myapplication.bluetooth

import kotlinx.coroutines.delay
import kotlinx.coroutines.launch
import kotlinx.coroutines.runBlocking
import kotlinx.coroutines.yield
import org.junit.Assert.assertEquals
import org.junit.Test

class FakeTransportTest {

    @Test
    fun `starts disconnected`() {
        val transport = FakeTransport()
        assertEquals(ConnectionState.Disconnected, transport.connectionState.value)
    }

    @Test
    fun `startAcceptLoop moves to Listening`() {
        val transport = FakeTransport()
        transport.startAcceptLoop()
        assertEquals(ConnectionState.Listening, transport.connectionState.value)
    }

    @Test
    fun `connectTo moves to Connected with the given device info`() = runBlocking {
        val transport = FakeTransport()
        transport.connectTo(address = "98:D3:31:FB:2A:C4", name = "HC-05")
        assertEquals(
            ConnectionState.Connected(deviceName = "HC-05", address = "98:D3:31:FB:2A:C4"),
            transport.connectionState.value,
        )
    }

    @Test
    fun `disconnect moves back to Disconnected`() = runBlocking {
        val transport = FakeTransport()
        transport.connectTo(address = "98:D3:31:FB:2A:C4", name = "HC-05")
        transport.disconnect()
        assertEquals(ConnectionState.Disconnected, transport.connectionState.value)
    }

    @Test
    fun `send records the line`() = runBlocking {
        val transport = FakeTransport()
        transport.send("MOVE,F")
        assertEquals(listOf("MOVE,F"), transport.sentLines)
    }

    @Test
    fun `emitIncoming is observable on incomingLines`() = runBlocking {
        val transport = FakeTransport()
        val received = mutableListOf<String>()
        val job = launch { transport.incomingLines.collect { received.add(it) } }
        yield()
        transport.emitIncoming("ROBOT,7,4,N")
        delay(10)
        job.cancel()
        assertEquals(listOf("ROBOT,7,4,N"), received)
    }
}
