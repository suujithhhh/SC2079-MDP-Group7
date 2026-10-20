package com.example.myapplication.bluetooth

sealed interface ConnectionState {
    data object Disconnected : ConnectionState
    data object Listening : ConnectionState
    data object Connecting : ConnectionState
    data class Connected(val deviceName: String, val address: String) : ConnectionState
}
