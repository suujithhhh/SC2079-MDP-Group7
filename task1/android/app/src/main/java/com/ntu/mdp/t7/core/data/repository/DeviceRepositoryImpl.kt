package com.ntu.mdp.t7.core.data.repository

import android.content.Context
import android.content.SharedPreferences
import com.ntu.mdp.t7.core.model.DiscoveredDevice

class DeviceRepositoryImpl(private val context: Context) : DeviceRepository {

    companion object {
        private const val PREFS_NAME = "arena_bluetooth_storage"
        private const val KEY_LAST_DEVICE_NAME = "last_device_name"
        private const val KEY_LAST_DEVICE_ADDRESS = "last_device_address"
    }

    private fun getPrefs(): SharedPreferences {
        return context.getSharedPreferences(PREFS_NAME, Context.MODE_PRIVATE)
    }

    override fun saveLastDevice(name: String, address: String) {
        getPrefs().edit()
            .putString(KEY_LAST_DEVICE_NAME, name)
            .putString(KEY_LAST_DEVICE_ADDRESS, address)
            .apply()
    }

    override fun loadLastDevice(): DiscoveredDevice? {
        val prefs = getPrefs()
        val name = prefs.getString(KEY_LAST_DEVICE_NAME, null)
        val address = prefs.getString(KEY_LAST_DEVICE_ADDRESS, null)

        return if (name != null && address != null) {
            DiscoveredDevice(name = name, address = address, isPaired = true)
        } else {
            null
        }
    }

    override fun clearLastDevice() {
        getPrefs().edit()
            .remove(KEY_LAST_DEVICE_NAME)
            .remove(KEY_LAST_DEVICE_ADDRESS)
            .apply()
    }
}
