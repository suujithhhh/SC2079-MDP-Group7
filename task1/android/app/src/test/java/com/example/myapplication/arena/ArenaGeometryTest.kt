package com.example.myapplication.arena

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
        // "Arena Controller - Portrait Screen.dc.html": obstacle 1 renders at
        // left=74,top=110 within a grid inset by a 2dp obstacle padding.
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
        // Facing N: box is 2 wide x 3 long. Rear-center is (7,4).
        // Since width is 2, bottom-left is x: 7 - 1 = 6, y: 4.
        // Top-left drawing cell is (6, 4+3-1=6). cellSize=36, gridSize=20:
        // left = 6*36 = 216, top = (20-1-6)*36 = 468.
        val offset = robotFootprintOffset(
            originX = 7, originY = 4, direction = Direction.NORTH,
            gridSize = gridSize, cellSize = cellSize, lengthCells = 3, widthCells = 2,
        )
        assertEquals(216f, offset.left)
        assertEquals(468f, offset.top)
    }

    @Test
    fun `robot footprint offset for EAST heading uses the swapped box dimensions and rear-center`() {
        // Facing E: box is 3 wide x 2 long. Rear-center is (7,4).
        // Since width is 2 (across y-axis), bottom-left is x: 7, y: 4 - 1 = 3.
        // Top-left drawing cell is (7, 3+2-1=4).
        // left = 7*36 = 252, top = (20-1-4)*36 = 540.
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

    @Test
    fun `offsetToGridCell does not clamp out-of-bounds points`() {
        assertEquals(GridCell(-1, gridSize - 1), offsetToGridCell(-cellSize, 0f, gridSize, cellSize))
        assertEquals(GridCell(0, -1), offsetToGridCell(0f, gridSize * cellSize, gridSize, cellSize))
    }

    @Test
    fun `nearestFreeCell returns the target itself when unoccupied`() {
        assertEquals(GridCell(5, 5), nearestFreeCell(GridCell(5, 5), gridSize) { false })
    }

    @Test
    fun `nearestFreeCell finds an adjacent cell when the target is occupied`() {
        val result = nearestFreeCell(GridCell(5, 5), gridSize) { it == GridCell(5, 5) }
        assertEquals(true, result != null)
        assertEquals(1, maxOf(kotlin.math.abs(result!!.x - 5), kotlin.math.abs(result.y - 5)))
    }

    @Test
    fun `nearestFreeCell only returns in-bounds cells near a grid corner`() {
        val result = nearestFreeCell(GridCell(0, 0), gridSize) { it == GridCell(0, 0) }
        assertEquals(true, result != null && result.x in 0 until gridSize && result.y in 0 until gridSize)
    }

    @Test
    fun `robot footprint cells for NORTH cover a 2-wide by 3-long block from the rear-center`() {
        val cells = robotFootprintCells(
            originX = 7, originY = 4, direction = Direction.NORTH, lengthCells = 3, widthCells = 2,
        )
        // bottom-left should be (6, 4)
        assertEquals(6, cells.size)
        assertEquals(true, GridCell(6, 4) in cells)
        assertEquals(true, GridCell(7, 6) in cells)
        assertEquals(false, GridCell(8, 4) in cells)
        assertEquals(false, GridCell(6, 7) in cells)
    }

    @Test
    fun `robot footprint cells for EAST cover a 3-wide by 2-long block from the rear-center`() {
        val cells = robotFootprintCells(
            originX = 7, originY = 4, direction = Direction.EAST, lengthCells = 3, widthCells = 2,
        )
        // bottom-left should be (7, 3)
        assertEquals(6, cells.size)
        assertEquals(true, GridCell(7, 3) in cells)
        assertEquals(true, GridCell(9, 4) in cells)
        assertEquals(false, GridCell(10, 3) in cells)
        assertEquals(false, GridCell(7, 5) in cells)
    }

    @Test
    fun `originFromTopLeftCell finds the rear-center origin`() {
        // NORTH: top-left drawing cell is (6, 6). 
        // sizeYCells = 3, width = 2.
        // bottom-left = (6, 6 - (3-1)) = (6, 4).
        // originX = 6 + (2/2) = 7. originY = 4.
        assertEquals(GridCell(7, 4), originFromTopLeftCell(GridCell(6, 6), sizeYCells = 3, direction = Direction.NORTH, widthCells = 2))
        
        // EAST: top-left drawing cell is (7, 4).
        // sizeYCells = 2, width = 2.
        // bottom-left = (7, 4 - (2-1)) = (7, 3).
        // originX = 7. originY = 3 + (2/2) = 4.
        assertEquals(GridCell(7, 4), originFromTopLeftCell(GridCell(7, 4), sizeYCells = 2, direction = Direction.EAST, widthCells = 2))
    }

    @Test
    fun `turnedPose displaces the origin and rotates heading for all 4 turn types from every heading`() {
        data class Case(val heading: Direction, val turn: TurnType, val dx: Int, val dy: Int, val newHeading: Direction)
        val cases = listOf(
            Case(Direction.NORTH, TurnType.FRONT_RIGHT, 3, 3, Direction.EAST),
            Case(Direction.NORTH, TurnType.FRONT_LEFT, -3, 3, Direction.WEST),
            Case(Direction.NORTH, TurnType.BACK_RIGHT, 3, -3, Direction.WEST),
            Case(Direction.NORTH, TurnType.BACK_LEFT, -3, -3, Direction.EAST),
            Case(Direction.EAST, TurnType.FRONT_RIGHT, 3, -3, Direction.SOUTH),
            Case(Direction.EAST, TurnType.FRONT_LEFT, 3, 3, Direction.NORTH),
            Case(Direction.EAST, TurnType.BACK_RIGHT, -3, -3, Direction.NORTH),
            Case(Direction.EAST, TurnType.BACK_LEFT, -3, 3, Direction.SOUTH),
            Case(Direction.SOUTH, TurnType.FRONT_RIGHT, -3, -3, Direction.WEST),
            Case(Direction.SOUTH, TurnType.FRONT_LEFT, 3, -3, Direction.EAST),
            Case(Direction.SOUTH, TurnType.BACK_RIGHT, -3, 3, Direction.EAST),
            Case(Direction.SOUTH, TurnType.BACK_LEFT, 3, 3, Direction.WEST),
            Case(Direction.WEST, TurnType.FRONT_RIGHT, -3, 3, Direction.NORTH),
            Case(Direction.WEST, TurnType.FRONT_LEFT, -3, -3, Direction.SOUTH),
            Case(Direction.WEST, TurnType.BACK_RIGHT, 3, 3, Direction.SOUTH),
            Case(Direction.WEST, TurnType.BACK_LEFT, 3, -3, Direction.NORTH),
        )
        for (case in cases) {
            val start = RobotPose(x = 10, y = 10, direction = case.heading)
            val result = turnedPose(start, case.turn, turnRadiusCells = 3)
            assertEquals("${case.heading} ${case.turn}: x", 10 + case.dx, result.x)
            assertEquals("${case.heading} ${case.turn}: y", 10 + case.dy, result.y)
            assertEquals("${case.heading} ${case.turn}: heading", case.newHeading, result.direction)
        }
    }
}
