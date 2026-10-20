package com.ntu.mdp.t7.core.bluetooth

import android.annotation.SuppressLint
import android.bluetooth.BluetoothAdapter
import android.bluetooth.BluetoothDevice
import android.bluetooth.BluetoothServerSocket
import android.bluetooth.BluetoothSocket
import android.content.BroadcastReceiver
import android.content.Context
import android.content.Intent
import android.content.IntentFilter
import android.util.Log
import androidx.core.content.ContextCompat
import com.ntu.mdp.t7.core.protocol.LineFramer
import java.io.Closeable
import java.io.IOException
import java.io.InputStream
import java.io.OutputStream
import java.util.UUID
import kotlinx.coroutines.CompletableDeferred
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.Job
import kotlinx.coroutines.SupervisorJob
import kotlinx.coroutines.cancel
import kotlinx.coroutines.CancellationException
import kotlinx.coroutines.delay
import kotlinx.coroutines.isActive
import kotlinx.coroutines.flow.MutableSharedFlow
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.SharedFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asSharedFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.launch
import kotlinx.coroutines.sync.Mutex
import kotlinx.coroutines.sync.withLock
import kotlinx.coroutines.withContext
import kotlinx.coroutines.withTimeoutOrNull

private const val TAG = "ClassicBtTransport"
private val SPP_UUID: UUID = UUID.fromString("00001101-0000-1000-8000-00805F9B34FB")
private const val MAX_CONNECTION_ATTEMPTS = 3
private const val BONDING_TIMEOUT_MS = 30_000L

@SuppressLint("MissingPermission")
class ClassicBluetoothTransport(
    context: Context,
    private val adapter: BluetoothAdapter,
) : BluetoothTransport {

    private val appContext = context.applicationContext
    private val scope = CoroutineScope(SupervisorJob() + Dispatchers.IO)

    private val _connectionState = MutableStateFlow<ConnectionState>(ConnectionState.Disconnected)
    override val connectionState: StateFlow<ConnectionState> = _connectionState.asStateFlow()

    private val _incomingLines = MutableSharedFlow<String>(extraBufferCapacity = 64)
    override val incomingLines: SharedFlow<String> = _incomingLines.asSharedFlow()

    private val socketLock = Mutex()
    @Volatile private var activeSocket: BluetoothSocket? = null
    @Volatile private var activeOutput: OutputStream? = null
    @Volatile private var sessionJob: Job? = null
    @Volatile private var acceptJob: Job? = null
    @Volatile private var currentServerSocket: BluetoothServerSocket? = null

    override fun startAcceptLoop() {
        if (acceptJob?.isActive == true) return
        acceptJob = scope.launch {
            try {
                if (adapter.name != "MDPG7x") {
                    adapter.name = "MDPG7x"
                }
            } catch (_: Exception) { }
            while (isActive) {
                val serverSocket = try {
                    adapter.listenUsingRfcommWithServiceRecord("MDPG7x", SPP_UUID)
                } catch (e: IOException) {
                    Log.w(TAG, "Could not open server socket, retrying shortly", e)
                    delay(2000)
                    continue
                }
                currentServerSocket = serverSocket
                if (!isActive) {
                    closeQuietly(serverSocket)
                    return@launch
                }
                if (_connectionState.value == ConnectionState.Disconnected) {
                    _connectionState.value = ConnectionState.Listening
                }
                val socket = try {
                    serverSocket.accept()
                } catch (e: IOException) {
                    Log.w(TAG, "Accept loop stopped", e)
                    null
                } finally {
                    closeQuietly(serverSocket)
                }
                if (socket != null) {
                    adoptSocket(socket)
                }
            }
        }
    }

    override suspend fun connectTo(address: String, name: String) {
        _connectionState.value = ConnectionState.Connecting
        withContext(Dispatchers.IO) {
            val device = adapter.getRemoteDevice(address)
            if (adapter.isDiscovering) {
                adapter.cancelDiscovery()
            }
            val wasAlreadyBonded = device.bondState == BluetoothDevice.BOND_BONDED
            if (!wasAlreadyBonded) {
                if (!awaitBonding(device)) {
                    Log.w(TAG, "Bonding with ${device.address} did not complete")
                    _connectionState.value =
                        if (acceptJob?.isActive == true) ConnectionState.Listening else ConnectionState.Disconnected
                    return@withContext
                }
                delay(1000L)
            }
            for (attempt in 1..MAX_CONNECTION_ATTEMPTS) {
                var socket: BluetoothSocket? = null
                try {
                    socket = device.createRfcommSocketToServiceRecord(SPP_UUID)
                    val connected = withTimeoutOrNull(8000L) {
                        socket.connect()
                        true
                    }
                    if (connected == true) {
                        adoptSocket(socket)
                        return@withContext
                    }
                    closeQuietly(socket)
                    Log.w(TAG, "Connect attempt $attempt/$MAX_CONNECTION_ATTEMPTS timed out after 8s")
                } catch (e: CancellationException) {
                    closeQuietly(socket)
                    throw e
                } catch (e: Exception) {
                    closeQuietly(socket)
                    Log.w(TAG, "Connect attempt $attempt/$MAX_CONNECTION_ATTEMPTS failed", e)
                }
            }
            _connectionState.value =
                if (acceptJob?.isActive == true) ConnectionState.Listening else ConnectionState.Disconnected
        }
    }

    private suspend fun awaitBonding(device: BluetoothDevice): Boolean {
        val bonded = CompletableDeferred<Boolean>()
        val receiver = object : BroadcastReceiver() {
            override fun onReceive(receivedContext: Context?, intent: Intent?) {
                val changedDevice = intent?.bluetoothDeviceExtra() ?: return
                if (changedDevice.address != device.address) return
                when (intent.getIntExtra(BluetoothDevice.EXTRA_BOND_STATE, BluetoothDevice.BOND_NONE)) {
                    BluetoothDevice.BOND_BONDED -> bonded.complete(true)
                    BluetoothDevice.BOND_NONE -> bonded.complete(false)
                }
            }
        }
        ContextCompat.registerReceiver(
            appContext,
            receiver,
            IntentFilter(BluetoothDevice.ACTION_BOND_STATE_CHANGED),
            ContextCompat.RECEIVER_EXPORTED,
        )
        try {
            if (device.bondState == BluetoothDevice.BOND_NONE) {
                Log.i(TAG, "Requesting pairing with ${device.address}")
                if (!device.createBond()) {
                    Log.w(TAG, "createBond() returned false for ${device.address}")
                    return false
                }
            }
            return withTimeoutOrNull(BONDING_TIMEOUT_MS) { bonded.await() } ?: false
        } finally {
            try {
                appContext.unregisterReceiver(receiver)
            } catch (e: IllegalArgumentException) { }
        }
    }

    private suspend fun adoptSocket(socket: BluetoothSocket) {
        socketLock.withLock {
            closeActiveSessionLocked()
            activeSocket = socket
            activeOutput = socket.outputStream
            _connectionState.value = ConnectionState.Connected(
                deviceName = socket.remoteDevice.name ?: "Unknown device",
                address = socket.remoteDevice.address,
            )
            Log.i(TAG, "Adopted socket from ${socket.remoteDevice.address}, starting read loop")
            sessionJob = scope.launch { readLoop(socket, socket.inputStream) }
        }
    }

    private suspend fun readLoop(socket: BluetoothSocket, input: InputStream) {
        val framer = LineFramer()
        val bytes = ByteArray(1024)
        try {
            while (true) {
                val count = input.read(bytes)
                if (count == -1) {
                    Log.i(TAG, "read() returned -1 (remote closed the stream)")
                    break
                }
                val chunk = String(bytes, 0, count, Charsets.UTF_8)
                Log.d(TAG, "read $count byte(s): ${chunk.replace("\r", "\\r").replace("\n", "\\n")}")
                val lines = framer.feed(chunk).toMutableList()
                if (lines.isEmpty() && input.available() == 0) {
                    framer.flushPending()?.let { lines.add(it) }
                }
                lines.forEach { line ->
                    if (line.isNotBlank()) {
                        Log.d(TAG, "framed line: $line")
                        _incomingLines.emit(line)
                    }
                }
            }
        } catch (e: IOException) {
            Log.w(TAG, "Connection lost", e)
        } finally {
            socketLock.withLock { closeSessionIfCurrentLocked(socket) }
        }
    }

    override suspend fun send(line: String) {
        val (socket, output) = socketLock.withLock { activeSocket to activeOutput }
        if (socket == null || output == null) return
        withContext(Dispatchers.IO) {
            try {
                output.write((line + "\n").toByteArray(Charsets.UTF_8))
                output.flush()
            } catch (e: Exception) {
                Log.w(TAG, "Send failed", e)
                socketLock.withLock { closeSessionIfCurrentLocked(socket) }
            }
        }
    }

    override fun disconnect() {
        scope.launch { socketLock.withLock { closeActiveSessionLocked() } }
    }

    override fun shutdown() {
        acceptJob?.cancel()
        closeQuietly(currentServerSocket)
        closeQuietly(activeSocket)
        scope.cancel()
    }

    private fun closeActiveSessionLocked() {
        sessionJob?.cancel()
        sessionJob = null
        closeQuietly(activeSocket)
        activeSocket = null
        activeOutput = null
        _connectionState.value =
            if (acceptJob?.isActive == true) ConnectionState.Listening else ConnectionState.Disconnected
    }

    private fun closeSessionIfCurrentLocked(socket: BluetoothSocket) {
        if (activeSocket !== socket) return
        closeActiveSessionLocked()
    }

    private fun closeQuietly(closeable: Closeable?) {
        try {
            closeable?.close()
        } catch (e: IOException) { }
    }
}
