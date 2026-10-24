package com.ntu.mdp.t7.core.model

import org.junit.Assert.assertEquals
import org.junit.Test

class ArenaModelsTest {

    @Test
    fun `null rssi is treated as weak`() {
        assertEquals(SignalQuality.WEAK, signalQualityFor(null))
    }

    @Test
    fun `rssi at or above -60 dBm is strong`() {
        assertEquals(SignalQuality.STRONG, signalQualityFor(-60))
        assertEquals(SignalQuality.STRONG, signalQualityFor(-42))
    }

    @Test
    fun `rssi below -60 dBm is weak`() {
        assertEquals(SignalQuality.WEAK, signalQualityFor(-61))
        assertEquals(SignalQuality.WEAK, signalQualityFor(-78))
    }

    @Test
    fun `nextObstacleId returns 1 when no obstacles exist`() {
        assertEquals(1, nextObstacleId(emptyList()))
    }

    @Test
    fun `nextObstacleId returns the next integer after the highest existing id when ids are contiguous`() {
        assertEquals(4, nextObstacleId(listOf(1, 2, 3)))
    }

    @Test
    fun `nextObstacleId fills the smallest available gap`() {
        assertEquals(2, nextObstacleId(listOf(1, 3, 4)))
    }

    @Test
    fun `obstaclesMissingFaces returns only the faceless obstacles`() {
        val obstacles = listOf(
            Obstacle(id = 1, x = 0, y = 0, face = Direction.NORTH),
            Obstacle(id = 2, x = 1, y = 1),
        )
        assertEquals(listOf(obstacles[1]), obstaclesMissingFaces(obstacles))
    }

    @Test
    fun `obstaclesMissingFaces returns empty when every obstacle has a face`() {
        val obstacles = listOf(Obstacle(id = 1, x = 0, y = 0, face = Direction.NORTH))
        assertEquals(emptyList<Obstacle>(), obstaclesMissingFaces(obstacles))
    }
}
