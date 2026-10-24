package com.ntu.mdp.t7.feature.task1_arena

import com.ntu.mdp.t7.core.model.Obstacle
import org.junit.Assert.assertEquals
import org.junit.Test

class ObstacleEditingTest {

    @Test
    fun `resolveDropCell excludes the moving obstacle by id, allowing it to drop back on its own cell`() {
        val obstacles = listOf(Obstacle(id = 1, x = 5, y = 5))
        val result = resolveDropCell(
            obstacles = obstacles,
            excludingId = 1,
            robotFootprint = emptySet(),
            gridSize = 20,
            target = GridCell(5, 5),
        )
        assertEquals(GridCell(5, 5), result)
    }

    @Test
    fun `resolveDropCell snaps away when the target is occupied by a different obstacle`() {
        val obstacles = listOf(Obstacle(id = 1, x = 5, y = 5), Obstacle(id = 2, x = 3, y = 3))
        val result = resolveDropCell(
            obstacles = obstacles,
            excludingId = 2,
            robotFootprint = emptySet(),
            gridSize = 20,
            target = GridCell(5, 5),
        )
        assertEquals(false, result == GridCell(5, 5))
    }

    @Test
    fun `resolveDropCell clamps an out-of-bounds target into the grid`() {
        val result = resolveDropCell(
            obstacles = emptyList(),
            excludingId = null,
            robotFootprint = emptySet(),
            gridSize = 20,
            target = GridCell(-3, 25),
        )
        assertEquals(true, result.x in 0 until 20 && result.y in 0 until 20)
    }

    @Test
    fun `resolveDropCell avoids the robot footprint`() {
        val footprint = setOf(GridCell(5, 5))
        val result = resolveDropCell(
            obstacles = emptyList(),
            excludingId = null,
            robotFootprint = footprint,
            gridSize = 20,
            target = GridCell(5, 5),
        )
        assertEquals(false, result == GridCell(5, 5))
    }
}
