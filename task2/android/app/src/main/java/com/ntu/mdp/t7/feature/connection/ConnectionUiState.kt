package com.ntu.mdp.t7.feature.connection

import com.ntu.mdp.t7.core.model.DevicePickerUiState
import com.ntu.mdp.t7.core.model.DiscoveredDevice
import com.ntu.mdp.t7.core.model.LinkState
import com.ntu.mdp.t7.core.model.StatusMessage

data class ConnectionUiState(
    val link: LinkState = LinkState.Idle,
    val savedDevice: DiscoveredDevice? = null,
    val devicePicker: DevicePickerUiState = DevicePickerUiState(),
    val statusLog: List<StatusMessage> = emptyList(),
    val rawProtocolLog: List<String> = emptyList(),
)
