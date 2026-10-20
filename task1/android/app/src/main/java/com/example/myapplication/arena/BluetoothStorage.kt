package com.example.myapplication.arena

import android.content.Context
import android.content.SharedPreferences

/**
 * Utility for persisting the last connected Bluetooth device for "Quick Connect".
 */
object BluetoothStorage {

    private const val PREFS_NAME = "arena_bluetooth_storage"
    private const val KEY_LAST_DEVICE_NAME = "last_device_name"
    private const val KEY_LAST_DEVICE_ADDRESS = "last_device_address"

    private fun getPrefs(context: Context): SharedPreferences {
        return context.getSharedPreferences(PREFS_NAME, Context.MODE_PRIVATE)
    }

    /**
     * Saves the last connected device info.
     */
    fun saveLastDevice(context: Context, name: String, address: String) {
        val prefs = getPrefs(context)
        prefs.edit()
            .putString(KEY_LAST_DEVICE_NAME, name)
            .putString(KEY_LAST_DEVICE_ADDRESS, address)
            .apply()
    }

    /**
     * Loads the last connected device info, or null if none is saved.
     */
    fun loadLastDevice(context: Context): DiscoveredDevice? {
        val prefs = getPrefs(context)
        val name = prefs.getString(KEY_LAST_DEVICE_NAME, null)
        val address = prefs.getString(KEY_LAST_DEVICE_ADDRESS, null)
        
        return if (name != null && address != null) {
            DiscoveredDevice(name = name, address = address, isPaired = true)
        } else {
            null
        }
    }

    /**
     * Clears the saved device info.
     */
    fun clearLastDevice(context: Context) {
        val prefs = getPrefs(context)
        prefs.edit()
            .remove(KEY_LAST_DEVICE_NAME)
            .remove(KEY_LAST_DEVICE_ADDRESS)
            .apply()
    }
}
