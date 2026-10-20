package com.example.myapplication.bluetooth

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
import com.example.myapplication.protocol.LineFramer
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

// Pairing needs a human to notice the system prompt and confirm it, which routinely takes far
// longer than the 8s RFCOMM connect timeout below — this timeout is only a safety net.
private const val BONDING_TIMEOUT_MS = 30_000L

/**
 * Real Bluetooth Classic transport. The AMD tool connects *to* this app, so an accept loop
 * must always be listening; the GUI must also be able to connect *out* to a discovered
 * device. Whichever wins hands its socket to the same single connected-session handler.
 * Only verifiable on a physical device — Android emulators have no Bluetooth radio.
 */
@SuppressLint("MissingPermission") // Callers must check BluetoothPermissions.allGranted() first.
class ClassicBluetoothTransport(
    context: Context,
    private val adapter: BluetoothAdapter,
) : BluetoothTransport {

    // Application context so this outlives the Activity that happened to construct it (this
    // transport is owned by ArenaViewModel, which survives configuration changes).
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
        try {
            if (adapter.name != "MDPG7x") {
                adapter.name = "MDPG7x"
            }
        } catch (_: Exception) {
            // Permission or adapter state issue
        }
        acceptJob = scope.launch {
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
                // Loop restarts (while still active) so a fresh accept() is posted after every
                // disconnect — this is what makes an AMD-initiated reconnect work.
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
            // A never-before-connected device is BOND_NONE. createRfcommSocketToServiceRecord()
            // + connect() *can* trigger pairing implicitly, but the system pairing prompt
            // routinely takes longer than the 8s-per-attempt connect loop below gives it — that
            // loop would close() the socket (and with it, the in-flight bonding request) before
            // the user ever gets to respond. So bonding is driven explicitly first, on its own
            // timeout, and only handed off to the RFCOMM connect loop once BOND_BONDED.
            val wasAlreadyBonded = device.bondState == BluetoothDevice.BOND_BONDED
            if (!wasAlreadyBonded) {
                if (!awaitBonding(device)) {
                    Log.w(TAG, "Bonding with ${device.address} did not complete")
                    _connectionState.value =
                        if (acceptJob?.isActive == true) ConnectionState.Listening else ConnectionState.Disconnected
                    return@withContext
                }
                // The Bluetooth stack tears down and re-negotiates the ACL link right after a
                // fresh bond completes; opening an RFCOMM socket in that window reliably times
                // out. Give it a moment to settle before the first connect attempt.
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

    /**
     * Requests pairing (if not already in progress) and suspends until [device] reaches
     * BOND_BONDED, BOND_NONE (pairing declined/failed), or [BONDING_TIMEOUT_MS] elapses.
     * Returns true only on BOND_BONDED.
     */
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
            } catch (e: IllegalArgumentException) {
                // Already unregistered.
            }
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

    /**
     * Every read()/line is logged at the socket boundary (not just via the status-log
     * "[Received] ..." fallback in ArenaViewModel, which only fires once a line is fully
     * framed) so `adb logcat -s ClassicBtTransport` can tell, independent of app logic,
     * whether bytes are landing on this socket at all — the question that "Connected but
     * nothing shows up" reports can't answer from the UI alone.
     */
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

    /**
     * Same teardown as [closeActiveSessionLocked], but only if [socket] is still the active
     * session. Guards the failure-path callers (readLoop's finally, send's error handler)
     * against tearing down a newer session that has already replaced this one via
     * [adoptSocket] by the time their own I/O failure is observed. Must be called while
     * holding [socketLock].
     */
    private fun closeSessionIfCurrentLocked(socket: BluetoothSocket) {
        if (activeSocket !== socket) return
        closeActiveSessionLocked()
    }

    private fun closeQuietly(closeable: Closeable?) {
        try {
            closeable?.close()
        } catch (e: IOException) {
            // Already gone; nothing to do.
        }
    }
}
