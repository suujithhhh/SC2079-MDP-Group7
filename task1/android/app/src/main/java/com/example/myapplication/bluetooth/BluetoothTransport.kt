package com.example.myapplication.bluetooth

import kotlinx.coroutines.flow.SharedFlow
import kotlinx.coroutines.flow.StateFlow

/**
 * Bluetooth Classic session, abstracted so the app can run against [FakeTransport]
 * (previews, emulator, no adapter/permissions) or [ClassicBluetoothTransport] (physical
 * device) interchangeably.
 */
interface BluetoothTransport {
    val connectionState: StateFlow<ConnectionState>
    val incomingLines: SharedFlow<String>

    /** Starts (or restarts) the accept loop so an inbound connection (e.g. from the AMD tool) can be received. */
    fun startAcceptLoop()

    /** GUI-initiated outward connection to a discovered device. */
    suspend fun connectTo(address: String, name: String)

    /** Sends one already-formatted command line; the `\n` terminator is added by the implementation. */
    suspend fun send(line: String)

    fun disconnect()

    /** Releases any background resources (coroutine scope, sockets). No-op by default. */
    fun shutdown() {}
}
