package com.ntu.mdp.t7.feature.connection.components

import androidx.compose.animation.animateColorAsState
import androidx.compose.foundation.background
import androidx.compose.foundation.clickable
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.size
import androidx.compose.foundation.layout.width
import androidx.compose.foundation.lazy.LazyColumn
import androidx.compose.foundation.lazy.items
import androidx.compose.foundation.shape.CircleShape
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material3.CircularProgressIndicator
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Surface
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.unit.dp
import androidx.compose.ui.window.Dialog
import androidx.compose.ui.window.DialogProperties
import com.ntu.mdp.t7.core.model.DiscoveredDevice
import com.ntu.mdp.t7.core.model.SignalQuality
import com.ntu.mdp.t7.core.model.signalQualityFor
import com.ntu.mdp.t7.feature.task1_arena.components.PillButton
import com.ntu.mdp.t7.ui.theme.AppTheme
import com.ntu.mdp.t7.ui.theme.AppType
import com.ntu.mdp.t7.ui.theme.Dimens
import com.ntu.mdp.t7.ui.theme.MyApplicationTheme
import kotlinx.coroutines.delay

private val PopupWidth = 560.dp
private val ColumnsHeight = 320.dp

@Composable
fun DevicePickerSheet(
    isScanning: Boolean,
    devices: List<DiscoveredDevice>,
    onDeviceSelected: (DiscoveredDevice) -> Unit,
    onDismiss: () -> Unit,
) {
    Dialog(onDismissRequest = onDismiss, properties = DialogProperties(usePlatformDefaultWidth = false)) {
        Surface(
            modifier = Modifier.width(PopupWidth),
            shape = RoundedCornerShape(Dimens.RadiusMedium),
            color = MaterialTheme.colorScheme.surface,
        ) {
            DevicePickerSheetContent(
                isScanning = isScanning,
                devices = devices,
                onDeviceSelected = onDeviceSelected,
                onCancel = onDismiss,
            )
        }
    }
}

@Composable
fun DevicePickerSheetContent(
    isScanning: Boolean,
    devices: List<DiscoveredDevice>,
    onDeviceSelected: (DiscoveredDevice) -> Unit,
    onCancel: () -> Unit,
) {
    val paired = devices.filter { it.isPaired }
    val available = devices.filterNot { it.isPaired }
    Column(
        modifier = Modifier
            .fillMaxWidth()
            .padding(Dimens.Space16),
        verticalArrangement = Arrangement.spacedBy(Dimens.Space12),
    ) {
        Text(text = "Select a device", style = AppType.Title, color = MaterialTheme.colorScheme.onSurface)

        Box(
            modifier = Modifier
                .fillMaxWidth()
                .height(36.dp),
            contentAlignment = Alignment.CenterStart,
        ) {
            if (isScanning) {
                ScanningRow()
            }
        }

        Row(
            modifier = Modifier
                .fillMaxWidth()
                .height(ColumnsHeight),
            horizontalArrangement = Arrangement.spacedBy(Dimens.Space16),
        ) {
            DeviceColumn(
                title = "Paired",
                devices = paired,
                emptyLabel = "No paired devices",
                onDeviceSelected = onDeviceSelected,
                modifier = Modifier.weight(1f),
            )
            DeviceColumn(
                title = "Available",
                devices = available,
                emptyLabel = "No devices found",
                onDeviceSelected = onDeviceSelected,
                modifier = Modifier.weight(1f),
            )
        }

        PillButton(label = "Cancel", onClick = onCancel, modifier = Modifier.fillMaxWidth())
    }
}

@Composable
private fun DeviceColumn(
    title: String,
    devices: List<DiscoveredDevice>,
    emptyLabel: String,
    onDeviceSelected: (DiscoveredDevice) -> Unit,
    modifier: Modifier = Modifier,
) {
    Column(modifier = modifier, verticalArrangement = Arrangement.spacedBy(Dimens.Space8)) {
        DeviceSectionHeader(title)
        if (devices.isEmpty()) {
            Text(text = emptyLabel, style = AppType.Body, color = MaterialTheme.colorScheme.onSurfaceVariant)
        } else {
            LazyColumn(
                modifier = Modifier.weight(1f),
                verticalArrangement = Arrangement.spacedBy(Dimens.Space8),
            ) {
                items(devices, key = { it.address }) { device ->
                    DeviceRow(device = device) { onDeviceSelected(device) }
                }
            }
        }
    }
}

@Composable
private fun DeviceSectionHeader(label: String) {
    Text(
        text = label,
        style = AppType.Subtitle,
        color = MaterialTheme.colorScheme.onSurfaceVariant,
    )
}

@Composable
private fun ScanningRow() {
    Row(
        modifier = Modifier.fillMaxWidth(),
        horizontalArrangement = Arrangement.spacedBy(Dimens.Space12),
        verticalAlignment = Alignment.CenterVertically,
    ) {
        CircularProgressIndicator(modifier = Modifier.size(20.dp))
        Text(
            text = "Scanning for devices…",
            style = AppType.Body,
            color = MaterialTheme.colorScheme.onSurfaceVariant,
        )
    }
}

@Composable
private fun DeviceRow(device: DiscoveredDevice, onSelected: () -> Unit) {
    val colors = AppTheme.colors
    var flashed by remember(device.address) { mutableStateOf(false) }
    val backgroundColor by animateColorAsState(
        targetValue = if (flashed) MaterialTheme.colorScheme.surfaceVariant else MaterialTheme.colorScheme.surface,
        label = "deviceRowFlash",
    )
    LaunchedEffect(flashed) {
        if (flashed) {
            delay(Dimens.StateChangeDurationMs.toLong())
            onSelected()
        }
    }
    val signalColor = when (signalQualityFor(device.signalStrength)) {
        SignalQuality.STRONG -> colors.linkOk
        SignalQuality.WEAK -> colors.linkWorking
    }
    Row(
        modifier = Modifier
            .fillMaxWidth()
            .clip(RoundedCornerShape(Dimens.RadiusSmall))
            .background(backgroundColor)
            .clickable(enabled = !flashed) { flashed = true }
            .padding(horizontal = Dimens.Space8, vertical = Dimens.Space12),
        horizontalArrangement = Arrangement.spacedBy(Dimens.Space12),
        verticalAlignment = Alignment.CenterVertically,
    ) {
        Box(modifier = Modifier.size(10.dp).background(signalColor, CircleShape))
        Column(modifier = Modifier.weight(1f)) {
            Text(text = device.name, style = AppType.Subtitle, color = MaterialTheme.colorScheme.onSurface)
            Text(text = device.address, style = AppType.Mono, color = MaterialTheme.colorScheme.onSurfaceVariant)
        }
    }
}
