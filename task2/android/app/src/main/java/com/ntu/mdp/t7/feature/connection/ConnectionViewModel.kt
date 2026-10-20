package com.ntu.mdp.t7.feature.connection

import androidx.lifecycle.ViewModel
import androidx.lifecycle.viewModelScope
import com.ntu.mdp.t7.core.bluetooth.BluetoothTransport
import com.ntu.mdp.t7.core.bluetooth.ConnectionState
import com.ntu.mdp.t7.core.bluetooth.FakeTransport
import com.ntu.mdp.t7.core.data.repository.DeviceRepository
import com.ntu.mdp.t7.core.model.DevicePickerUiState
import com.ntu.mdp.t7.core.model.DiscoveredDevice
import com.ntu.mdp.t7.core.model.LinkState
import com.ntu.mdp.t7.core.model.SignalQuality
import com.ntu.mdp.t7.core.model.StatusLevel
import com.ntu.mdp.t7.core.model.StatusMessage
import com.ntu.mdp.t7.core.model.signalQualityFor
import java.text.SimpleDateFormat
import java.util.Date
import java.util.Locale
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.Job
import kotlinx.coroutines.delay
import kotlinx.coroutines.flow.MutableSharedFlow
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.SharedFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asSharedFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.flow.update
import kotlinx.coroutines.launch

private const val MAX_RECONNECT_ATTEMPTS = 10
private const val RECONNECT_DELAY_MS = 1500L

class ConnectionViewModel(
    initialTransport: BluetoothTransport = FakeTransport(),
    private val deviceRepository: DeviceRepository? = null,
) : ViewModel() {

    var transport: BluetoothTransport = initialTransport
        private set

    private val _uiState = MutableStateFlow(ConnectionUiState())
    val uiState: StateFlow<ConnectionUiState> = _uiState.asStateFlow()

    private val _scanNonce = MutableStateFlow(0)
    val scanNonce: StateFlow<Int> = _scanNonce.asStateFlow()

    private val _incomingLines = MutableSharedFlow<String>(extraBufferCapacity = 64)
    val incomingLines: SharedFlow<String> = _incomingLines.asSharedFlow()

    private val timeFormat = SimpleDateFormat("HH:mm:ss", Locale.US)
    private var pendingConnectSignal: SignalQuality? = null
    private var collectorJob: Job? = null
    private var reconnectJob: Job? = null

    init {
        attachTransport(initialTransport)
        viewModelScope.launch(Dispatchers.IO) {
            loadSavedDevice()
        }
    }

    private fun loadSavedDevice() {
        val saved = deviceRepository?.loadLastDevice()
        if (saved != null) {
            _uiState.update { it.copy(savedDevice = saved) }
        }
    }

    private fun attachTransport(t: BluetoothTransport) {
        collectorJob?.cancel()
        collectorJob = viewModelScope.launch {
            launch { t.connectionState.collect { handleConnectionState(it) } }
            launch { t.incomingLines.collect { line ->
                _incomingLines.emit(line)
                _uiState.update { state ->
                    state.copy(rawProtocolLog = (state.rawProtocolLog + "← $line").takeLast(100))
                }
            } }
        }
        t.startAcceptLoop()
    }

    fun upgradeTransport(newTransport: BluetoothTransport) {
        if (transport === newTransport) return
        val old = transport
        transport = newTransport
        attachTransport(newTransport)
        old.shutdown()
    }

    override fun onCleared() {
        transport.shutdown()
    }

    fun logOutbound(message: String) {
        _uiState.update {
            it.copy(rawProtocolLog = (it.rawProtocolLog + "→ $message").takeLast(100))
        }
    }

    fun addStatusLog(message: String, level: StatusLevel = StatusLevel.INFO) {
        _uiState.update {
            it.copy(statusLog = (it.statusLog + StatusMessage(
                time = timeFormat.format(Date()),
                message = message,
                level = level
            )).takeLast(100))
        }
    }

    fun onScanClick(pairedDevices: List<DiscoveredDevice> = emptyList()) {
        _uiState.update {
            it.copy(
                link = LinkState.Scanning,
                devicePicker = DevicePickerUiState(isVisible = true, isScanning = true, devices = pairedDevices)
            )
        }
        _scanNonce.update { it + 1 }
    }

    fun onDeviceFound(name: String, address: String, rssi: Int?, isPaired: Boolean = false) {
        _uiState.update { state ->
            val updated = state.devicePicker.devices.filterNot { it.address == address } +
                DiscoveredDevice(name = name, address = address, signalStrength = rssi, isPaired = isPaired)
            state.copy(devicePicker = state.devicePicker.copy(devices = updated))
        }
    }

    fun onScanFinished() {
        _uiState.update { state ->
            state.copy(devicePicker = state.devicePicker.copy(isScanning = false))
        }
    }

    fun onDeviceSelected(device: DiscoveredDevice) {
        pendingConnectSignal = signalQualityFor(device.signalStrength)
        _uiState.update { it.copy(devicePicker = DevicePickerUiState()) }
        connectDirectly(device.name, device.address)
    }

    fun onDevicePickerDismiss() {
        _uiState.update { state ->
            state.copy(
                devicePicker = DevicePickerUiState(),
                link = if (state.link == LinkState.Scanning) LinkState.Idle else state.link
            )
        }
    }

    fun onConnectClick() {
        when (val link = _uiState.value.link) {
            is LinkState.Lost -> {
                reconnectJob?.cancel()
                connectDirectly(link.deviceName, link.address)
            }
            else -> Unit
        }
    }

    fun connectDirectly(deviceName: String, address: String) {
        _uiState.update { it.copy(link = LinkState.Connecting(deviceName, address)) }
        viewModelScope.launch { transport.connectTo(address, deviceName) }
    }

    fun onDisconnectClick() {
        reconnectJob?.cancel()
        transport.disconnect()
        _uiState.update { state ->
            state.copy(
                link = LinkState.Idle,
                statusLog = (state.statusLog + StatusMessage(
                    time = timeFormat.format(Date()),
                    message = "Disconnected",
                    level = StatusLevel.INFO
                )).takeLast(100)
            )
        }
    }

    fun onQuickConnectClick() {
        val saved = _uiState.value.savedDevice ?: return
        connectDirectly(saved.name, saved.address)
    }

    fun onClearSavedDeviceClick() {
        deviceRepository?.clearLastDevice()
        _uiState.update { it.copy(savedDevice = null) }
    }

    fun onBluetoothPermissionDenied() {
        addStatusLog("Bluetooth permission denied. Enable it in Settings to use Bluetooth.", StatusLevel.ALERT)
    }

    private fun handleConnectionState(state: ConnectionState) {
        when (state) {
            is ConnectionState.Connected -> {
                reconnectJob?.cancel()
                reconnectJob = null
                deviceRepository?.saveLastDevice(state.deviceName, state.address)
                _uiState.update { current ->
                    val signal = pendingConnectSignal ?: SignalQuality.WEAK
                    pendingConnectSignal = null
                    val newSaved = DiscoveredDevice(name = state.deviceName, address = state.address, isPaired = true)
                    current.copy(
                        link = LinkState.Connected(state.deviceName, state.address, signal),
                        savedDevice = newSaved,
                        statusLog = (current.statusLog + StatusMessage(
                            time = timeFormat.format(Date()),
                            message = "Connected to ${state.deviceName}",
                            level = StatusLevel.INFO
                        )).takeLast(100)
                    )
                }
            }
            ConnectionState.Disconnected, ConnectionState.Listening -> {
                if (reconnectJob?.isActive == true) return
                val previousLink = _uiState.value.link
                _uiState.update { current ->
                    when (val link = current.link) {
                        is LinkState.Connected -> current.copy(
                            link = LinkState.Lost(link.deviceName, link.address),
                            statusLog = (current.statusLog + StatusMessage(
                                time = timeFormat.format(Date()),
                                message = "Connection lost. Reconnecting…",
                                level = StatusLevel.ALERT
                            )).takeLast(100)
                        )
                        is LinkState.Connecting -> current.copy(
                            link = LinkState.Idle,
                            statusLog = (current.statusLog + StatusMessage(
                                time = timeFormat.format(Date()),
                                message = "Connection failed",
                                level = StatusLevel.ALERT
                            )).takeLast(100)
                        )
                        else -> current
                    }
                }
                if (previousLink is LinkState.Connected) {
                    startReconnectLoop(previousLink.deviceName, previousLink.address)
                }
            }
            ConnectionState.Connecting -> Unit
        }
    }

    private fun startReconnectLoop(deviceName: String, address: String) {
        reconnectJob?.cancel()
        reconnectJob = viewModelScope.launch {
            repeat(MAX_RECONNECT_ATTEMPTS) {
                delay(RECONNECT_DELAY_MS)
                _uiState.update { it.copy(link = LinkState.Connecting(deviceName, address)) }
                transport.connectTo(address, deviceName)
                if (transport.connectionState.value is ConnectionState.Connected) return@launch
                _uiState.update { it.copy(link = LinkState.Lost(deviceName, address)) }
            }
            addStatusLog("Could not reconnect after $MAX_RECONNECT_ATTEMPTS attempts — tap Connect to retry", StatusLevel.ALERT)
        }
    }
}
