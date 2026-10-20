package com.ntu.mdp.t7.core.data.repository

import com.ntu.mdp.t7.core.model.DiscoveredDevice

/**
 * Repository interface for persisting and loading the last connected Bluetooth device.
 */
interface DeviceRepository {
    fun saveLastDevice(name: String, address: String)
    fun loadLastDevice(): DiscoveredDevice?
    fun clearLastDevice()
}
