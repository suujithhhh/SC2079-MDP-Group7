package com.ntu.mdp.t7.core.data.repository

import com.ntu.mdp.t7.core.model.ObstaclePreset

/**
 * Repository interface for persisting and loading obstacle presets.
 */
interface PresetRepository {
    fun loadPresets(): List<ObstaclePreset?>
    fun savePreset(preset: ObstaclePreset)
    fun clearPreset(slotIndex: Int)
}
