package com.ntu.mdp.t7.feature.task1_arena

import com.ntu.mdp.t7.core.model.ArenaConfig
import com.ntu.mdp.t7.core.model.Direction
import org.junit.Assert.assertEquals
import org.junit.Test

class ArenaGeometryTest {

    private val gridSize = ArenaConfig.GRID_SIZE
    private val cellSize = ArenaConfig.CELL_SIZE_DP.toFloat()

    @Test
    fun `origin cell (0,0) sits at bottom-left of the drawing area`() {
        val offset = gridToOffset(0, 0, gridSize, cellSize)
        assertEquals(0f, offset.left)
        assertEquals((gridSize - 1) * cellSize, offset.top)
    }

    @Test
    fun `top-left grid cell (0, gridSize-1) sits at drawing origin`() {
        val offset = gridToOffset(0, gridSize - 1, gridSize, cellSize)
        assertEquals(0f, offset.left)
        assertEquals(0f, offset.top)
    }

    @Test
    fun `matches obstacle 1 position from the design mockup`() {
        val offset = gridToOffset(2, 16, gridSize, cellSize)
        assertEquals(72f, offset.left)
        assertEquals(108f, offset.top)
    }

    @Test
    fun `footprintBox swaps width and length between N-S and E-W headings`() {
        assertEquals(2 to 3, footprintBox(Direction.NORTH, lengthCells = 3, widthCells = 2))
        assertEquals(2 to 3, footprintBox(Direction.SOUTH, lengthCells = 3, widthCells = 2))
        assertEquals(3 to 2, footprintBox(Direction.EAST, lengthCells = 3, widthCells = 2))
        assertEquals(3 to 2, footprintBox(Direction.WEST, lengthCells = 3, widthCells = 2))
    }

    @Test
    fun `robot footprint offset for NORTH heading treats the rear-center as the box's origin`() {
        val offset = robotFootprintOffset(
            originX = 7, originY = 4, direction = Direction.NORTH,
            gridSize = gridSize, cellSize = cellSize, lengthCells = 3, widthCells = 2,
        )
        assertEquals(216f, offset.left)
        assertEquals(468f, offset.top)
    }

    @Test
    fun `robot footprint offset for EAST heading uses the swapped box dimensions and rear-center`() {
        val offset = robotFootprintOffset(
            originX = 7, originY = 4, direction = Direction.EAST,
            gridSize = gridSize, cellSize = cellSize, lengthCells = 3, widthCells = 2,
        )
        assertEquals(252f, offset.left)
        assertEquals(540f, offset.top)
    }

    @Test
    fun `offsetToGridCell is the inverse of gridToOffset for every cell`() {
        for (x in 0 until gridSize) {
            for (y in 0 until gridSize) {
                val offset = gridToOffset(x, y, gridSize, cellSize)
                assertEquals(GridCell(x, y), offsetToGridCell(offset.left, offset.top, gridSize, cellSize))
            }
        }
    }
}
