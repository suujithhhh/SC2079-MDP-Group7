package com.example.myapplication.bluetooth

import android.annotation.SuppressLint
import android.bluetooth.BluetoothAdapter
import android.bluetooth.BluetoothDevice
import android.content.BroadcastReceiver
import android.content.Context
import android.content.Intent
import android.content.IntentFilter
import android.os.Build
import android.util.Log
import androidx.core.content.ContextCompat

private const val TAG = "BluetoothDiscovery"

/**
 * Wraps [BluetoothAdapter.startDiscovery] behind a broadcast receiver, surfacing each
 * nearby device found and when the scan (Android caps this at ~12s) finishes. Also watches
 * [BluetoothDevice.ACTION_BOND_STATE_CHANGED] so a device paired mid-scan (e.g. via the
 * system pairing prompt triggered when connecting to it) is re-surfaced with
 * `isPaired = true` without needing a new scan. Caller must check
 * [BluetoothPermissions.allGranted] before calling [start].
 */
@SuppressLint("MissingPermission") // Caller must check BluetoothPermissions.allGranted() first.
class BluetoothDiscovery(private val context: Context, private val adapter: BluetoothAdapter) {

    private var receiver: BroadcastReceiver? = null

    fun start(onEvent: (DiscoveryEvent) -> Unit) {
        stop()

        // Surface already-bonded/paired devices immediately
        try {
            val bonded = adapter.bondedDevices.orEmpty()
            Log.d(TAG, "bonded devices: ${bonded.map { it.name ?: it.address }}")
            bonded.forEach { device ->
                onEvent(
                    DiscoveryEvent.DeviceFound(
                        name = device.name ?: device.address,
                        address = device.address,
                        rssi = null,
                        isPaired = true,
                    ),
                )
            }
        } catch (e: Exception) {
            Log.w(TAG, "Could not read bonded devices", e)
        }

        val filter = IntentFilter().apply {
            addAction(BluetoothDevice.ACTION_FOUND)
            addAction(BluetoothDevice.ACTION_BOND_STATE_CHANGED)
            addAction(BluetoothAdapter.ACTION_DISCOVERY_FINISHED)
        }
        val newReceiver = object : BroadcastReceiver() {
            override fun onReceive(receivedContext: Context?, intent: Intent?) {
                when (intent?.action) {
                    BluetoothDevice.ACTION_FOUND -> {
                        val device = intent.bluetoothDeviceExtra()
                        if (device == null) {
                            Log.w(TAG, "ACTION_FOUND with no EXTRA_DEVICE")
                            return
                        }
                        val nameExtra = intent.getStringExtra(BluetoothDevice.EXTRA_NAME)
                        val rssi = intent.getShortExtra(BluetoothDevice.EXTRA_RSSI, Short.MIN_VALUE)
                        val deviceName = device.name ?: nameExtra ?: device.address
                        val isPaired = device.bondState == BluetoothDevice.BOND_BONDED
                        Log.d(
                            TAG,
                            "ACTION_FOUND: $deviceName (${device.address}), " +
                                "bondState=${device.bondState}, rssi=$rssi",
                        )
                        onEvent(
                            DiscoveryEvent.DeviceFound(
                                name = deviceName,
                                address = device.address,
                                rssi = if (rssi == Short.MIN_VALUE) null else rssi.toInt(),
                                isPaired = isPaired,
                            ),
                        )
                    }
                    BluetoothDevice.ACTION_BOND_STATE_CHANGED -> {
                        val device = intent.bluetoothDeviceExtra() ?: return
                        val bondState = intent.getIntExtra(
                            BluetoothDevice.EXTRA_BOND_STATE,
                            BluetoothDevice.BOND_NONE,
                        )
                        Log.d(TAG, "ACTION_BOND_STATE_CHANGED: ${device.address} -> $bondState")
                        if (bondState == BluetoothDevice.BOND_BONDED) {
                            onEvent(
                                DiscoveryEvent.DeviceFound(
                                    name = device.name ?: device.address,
                                    address = device.address,
                                    rssi = null,
                                    isPaired = true,
                                ),
                            )
                        }
                    }
                    BluetoothAdapter.ACTION_DISCOVERY_FINISHED -> {
                        Log.d(TAG, "ACTION_DISCOVERY_FINISHED")
                        onEvent(DiscoveryEvent.Finished)
                    }
                }
            }
        }
        receiver = newReceiver
        ContextCompat.registerReceiver(context, newReceiver, filter, ContextCompat.RECEIVER_EXPORTED)
        val started = adapter.startDiscovery()
        Log.d(TAG, "startDiscovery() returned $started")
        if (!started) {
            onEvent(DiscoveryEvent.Finished)
        }
    }

    fun stop() {
        if (adapter.isDiscovering) {
            Log.d(TAG, "cancelDiscovery()")
            adapter.cancelDiscovery()
        }
        receiver?.let {
            try {
                context.unregisterReceiver(it)
            } catch (e: IllegalArgumentException) {
                // Already unregistered.
            }
        }
        receiver = null
    }
}

/** Shared with [ClassicBluetoothTransport], which also watches bond-state changes. */
internal fun Intent.bluetoothDeviceExtra(): BluetoothDevice? =
    if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.TIRAMISU) {
        getParcelableExtra(BluetoothDevice.EXTRA_DEVICE, BluetoothDevice::class.java)
    } else {
        @Suppress("DEPRECATION")
        getParcelableExtra(BluetoothDevice.EXTRA_DEVICE)
    }
