package com.ntu.mdp.t7.core.bluetooth

sealed interface DiscoveryEvent {
    data class DeviceFound(
        val name: String,
        val address: String,
        val rssi: Int?,
        val isPaired: Boolean = false,
    ) : DiscoveryEvent
    data object Finished : DiscoveryEvent
}
