package com.example.myapplication.bluetooth

sealed interface DiscoveryEvent {
    data class DeviceFound(
        val name: String,
        val address: String,
        val rssi: Int?,
        val isPaired: Boolean = false,
    ) : DiscoveryEvent
    data object Finished : DiscoveryEvent
}
