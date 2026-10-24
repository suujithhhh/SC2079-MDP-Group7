package com.ntu.mdp.t7.core.bluetooth

import kotlinx.coroutines.flow.SharedFlow
import kotlinx.coroutines.flow.StateFlow

interface BluetoothTransport {
    val connectionState: StateFlow<ConnectionState>
    val incomingLines: SharedFlow<String>

    fun startAcceptLoop()
    suspend fun connectTo(address: String, name: String)
    suspend fun send(line: String)
    fun disconnect()
    fun shutdown() {}
}
