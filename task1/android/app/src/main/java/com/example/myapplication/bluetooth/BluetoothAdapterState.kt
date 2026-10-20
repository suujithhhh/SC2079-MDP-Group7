package com.example.myapplication.bluetooth

import android.Manifest
import android.bluetooth.BluetoothAdapter
import android.bluetooth.BluetoothManager
import android.content.BroadcastReceiver
import android.content.Context
import android.content.Intent
import android.content.IntentFilter
import android.content.pm.PackageManager
import android.os.Build
import androidx.activity.compose.rememberLauncherForActivityResult
import androidx.activity.result.contract.ActivityResultContracts
import androidx.compose.runtime.Composable
import androidx.compose.runtime.DisposableEffect
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.runtime.State
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.ui.platform.LocalContext
import androidx.core.content.ContextCompat
import kotlinx.coroutines.delay
import kotlinx.coroutines.isActive

/** How often [rememberBluetoothEnabled] polls [BluetoothAdapter.isEnabled] as a fallback
 * for when [BluetoothAdapter.ACTION_STATE_CHANGED] arrives late (a known OEM quirk). */
private const val BLUETOOTH_STATE_POLL_INTERVAL_MS = 2000L

/**
 * Live-observes whether the device's Bluetooth radio is turned on. Reading
 * [BluetoothAdapter.isEnabled] requires no runtime permission on any API level — only
 * scanning/connecting do. See the team design notes.
 */
@Composable
fun rememberBluetoothEnabled(): State<Boolean> {
    val context = LocalContext.current
    val adapter = remember {
        (context.getSystemService(Context.BLUETOOTH_SERVICE) as? BluetoothManager)?.adapter
    }
    val enabled = remember { mutableStateOf(adapter?.isEnabled == true) }

    DisposableEffect(adapter) {
        if (adapter == null) {
            enabled.value = false
            return@DisposableEffect onDispose {}
        }
        val receiver = object : BroadcastReceiver() {
            override fun onReceive(context: Context?, intent: Intent?) {
                enabled.value = adapter.isEnabled
            }
        }
        ContextCompat.registerReceiver(
            context,
            receiver,
            IntentFilter(BluetoothAdapter.ACTION_STATE_CHANGED),
            ContextCompat.RECEIVER_EXPORTED,
        )
        enabled.value = adapter.isEnabled
        onDispose { context.unregisterReceiver(receiver) }
    }

    LaunchedEffect(adapter) {
        if (adapter == null) return@LaunchedEffect
        while (isActive) {
            delay(BLUETOOTH_STATE_POLL_INTERVAL_MS)
            enabled.value = adapter.isEnabled
        }
    }

    return enabled
}

/**
 * Returns a callback that prompts the user to turn Bluetooth on. On API 31+, launching
 * [BluetoothAdapter.ACTION_REQUEST_ENABLE] requires the BLUETOOTH_CONNECT runtime
 * permission; this requests it first if needed, then launches the system enable dialog
 * once granted.
 */
@Composable
fun rememberEnableBluetoothLauncher(onPermissionDenied: () -> Unit = {}): () -> Unit {
    val context = LocalContext.current
    val enableBluetoothLauncher = rememberLauncherForActivityResult(
        contract = ActivityResultContracts.StartActivityForResult(),
    ) { /* bluetoothEnabled updates via the BroadcastReceiver in rememberBluetoothEnabled */ }
    val requestPermissionLauncher = rememberLauncherForActivityResult(
        contract = ActivityResultContracts.RequestPermission(),
    ) { granted ->
        if (granted) {
            enableBluetoothLauncher.launch(Intent(BluetoothAdapter.ACTION_REQUEST_ENABLE))
        } else {
            onPermissionDenied()
        }
    }

    return {
        val needsPermission = Build.VERSION.SDK_INT >= Build.VERSION_CODES.S &&
            ContextCompat.checkSelfPermission(context, Manifest.permission.BLUETOOTH_CONNECT) !=
            PackageManager.PERMISSION_GRANTED
        if (needsPermission) {
            requestPermissionLauncher.launch(Manifest.permission.BLUETOOTH_CONNECT)
        } else {
            enableBluetoothLauncher.launch(Intent(BluetoothAdapter.ACTION_REQUEST_ENABLE))
        }
    }
}

/**
 * Returns a callback that prompts the user to make this device discoverable to other
 * Bluetooth devices for [durationSeconds] (0 = no time limit, per
 * [BluetoothAdapter.ACTION_REQUEST_DISCOVERABLE] docs). Without this, the accept loop in
 * [ClassicBluetoothTransport] can still receive a connection from an already-paired/bonded
 * device, but the AMD tool's "Scan For Devices" (protocol-spec.md) will never find this
 * tablet in the first place, since merely holding an open [android.bluetooth.BluetoothServerSocket]
 * does not put the adapter into discoverable scan mode.
 *
 * Caller must check [BluetoothPermissions.allGranted] first — BLUETOOTH_ADVERTISE (API 31+,
 * included in [BluetoothPermissions.required]) is required to launch this intent, and this
 * function does not request it itself.
 */
@Composable
fun rememberRequestDiscoverableLauncher(durationSeconds: Int = 0): () -> Unit {
    val discoverableLauncher = rememberLauncherForActivityResult(
        contract = ActivityResultContracts.StartActivityForResult(),
    ) { /* No result to observe; scan-mode changes aren't surfaced back to the caller. */ }

    return {
        discoverableLauncher.launch(
            Intent(BluetoothAdapter.ACTION_REQUEST_DISCOVERABLE)
                .putExtra(BluetoothAdapter.EXTRA_DISCOVERABLE_DURATION, durationSeconds),
        )
    }
}
