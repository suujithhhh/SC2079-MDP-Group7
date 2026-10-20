package com.ntu.mdp.t7.core.data.repository

import android.content.Context
import android.content.SharedPreferences
import com.ntu.mdp.t7.core.model.Direction
import com.ntu.mdp.t7.core.model.Obstacle
import com.ntu.mdp.t7.core.model.ObstaclePreset
import org.json.JSONArray
import org.json.JSONObject

class PresetRepositoryImpl(private val context: Context) : PresetRepository {

    companion object {
        private const val PREFS_NAME = "arena_preset_storage"
        private const val SLOT_COUNT = 3
    }

    private fun getPrefs(): SharedPreferences {
        return context.getSharedPreferences(PREFS_NAME, Context.MODE_PRIVATE)
    }

    override fun loadPresets(): List<ObstaclePreset?> {
        val prefs = getPrefs()
        val result = mutableListOf<ObstaclePreset?>()

        for (slot in 0 until SLOT_COUNT) {
            val jsonString = prefs.getString("preset_$slot", null)
            if (jsonString.isNullOrEmpty()) {
                result.add(null)
            } else {
                try {
                    val jsonArray = JSONArray(jsonString)
                    val obstacles = mutableListOf<Obstacle>()
                    for (i in 0 until jsonArray.length()) {
                        val obj = jsonArray.getJSONObject(i)
                        val id = obj.getInt("id")
                        val x = obj.getInt("x")
                        val y = obj.getInt("y")
                        val faceCode = if (obj.has("face")) obj.getString("face").firstOrNull() else null
                        val face = faceCode?.let { Direction.fromCode(it) }
                        obstacles.add(Obstacle(id = id, x = x, y = y, face = face))
                    }
                    result.add(ObstaclePreset(slotIndex = slot, obstacles = obstacles))
                } catch (_: Exception) {
                    result.add(null)
                }
            }
        }
        return result
    }

    override fun savePreset(preset: ObstaclePreset) {
        val prefs = getPrefs()
        val jsonArray = JSONArray()

        for (obstacle in preset.obstacles) {
            val obj = JSONObject()
            obj.put("id", obstacle.id)
            obj.put("x", obstacle.x)
            obj.put("y", obstacle.y)
            obstacle.face?.let { obj.put("face", it.code.toString()) }
            jsonArray.put(obj)
        }

        prefs.edit().putString("preset_${preset.slotIndex}", jsonArray.toString()).apply()
    }

    override fun clearPreset(slotIndex: Int) {
        val prefs = getPrefs()
        prefs.edit().remove("preset_$slotIndex").apply()
    }
}
