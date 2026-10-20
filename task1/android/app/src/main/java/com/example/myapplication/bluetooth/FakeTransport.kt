package com.example.myapplication.bluetooth

import kotlinx.coroutines.flow.MutableSharedFlow
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.SharedFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asSharedFlow
import kotlinx.coroutines.flow.asStateFlow

/**
 * In-memory transport with no real Bluetooth I/O. Used by Compose previews and as the
 * default whenever no adapter/permissions are available (always true on an emulator).
 */
class FakeTransport : BluetoothTransport {

    private val _connectionState = MutableStateFlow<ConnectionState>(ConnectionState.Disconnected)
    override val connectionState: StateFlow<ConnectionState> = _connectionState.asStateFlow()

    private val _incomingLines = MutableSharedFlow<String>(extraBufferCapacity = 16)
    override val incomingLines: SharedFlow<String> = _incomingLines.asSharedFlow()

    private val _sentLines = mutableListOf<String>()
    val sentLines: List<String> get() = _sentLines

    /** When true, [connectTo] reports failure instead of succeeding — for testing retry/give-up behavior. */
    var failConnections: Boolean = false

    /** Number of times [connectTo] has been called — for testing retry-count behavior. */
    var connectAttempts: Int = 0
        private set

    override fun startAcceptLoop() {
        _connectionState.value = ConnectionState.Listening
    }

    override suspend fun connectTo(address: String, name: String) {
        connectAttempts++
        _connectionState.value = ConnectionState.Connecting
        _connectionState.value = if (failConnections) {
            ConnectionState.Disconnected
        } else {
            ConnectionState.Connected(deviceName = name, address = address)
        }
    }

    override suspend fun send(line: String) {
        _sentLines.add(line)
    }

    override fun disconnect() {
        _connectionState.value = ConnectionState.Disconnected
    }

    /** Test/demo hook: pretend a line arrived from the remote side. */
    suspend fun emitIncoming(line: String) {
        _incomingLines.emit(line)
    }
}
