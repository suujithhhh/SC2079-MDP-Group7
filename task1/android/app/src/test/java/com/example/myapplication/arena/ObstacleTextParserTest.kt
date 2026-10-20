package com.example.myapplication.arena

import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test

class ObstacleTextParserTest {

    private val gridSize = 20
    private val noBlocked = emptySet<GridCell>()

    private fun parse(text: String, blocked: Set<GridCell> = noBlocked) =
        parseObstacleText(text, gridSize, blocked)

    private val needFace = "end the line with a face letter: u, d, l or r"
    private val badFace = "face must be u, d, l or r"

    @Test
    fun `face letters map to wire directions - u d l r to N S W E`() {
        assertEquals(Direction.NORTH, faceFromLetter('u'))
        assertEquals(Direction.SOUTH, faceFromLetter('d'))
        assertEquals(Direction.WEST, faceFromLetter('l'))
        assertEquals(Direction.EAST, faceFromLetter('r'))
        assertEquals(Direction.WEST, faceFromLetter('L'))
        assertNull(faceFromLetter('x'))
        assertNull(faceFromLetter('n'))
    }

    @Test
    fun `faceToLetter is the inverse of faceFromLetter`() {
        Direction.entries.forEach { assertEquals(it, faceFromLetter(faceToLetter(it))) }
    }

    @Test
    fun `task sheet example parses to ids 1 to 5 in line order - face letter ends the line`() {
        val result = parse("5,13l\n5,7d\n12,9r\n15,15d\n15,4u\n")

        assertTrue(result.isPlaceable)
        assertEquals(
            listOf(
                Obstacle(id = 1, x = 5, y = 13, face = Direction.WEST),
                Obstacle(id = 2, x = 5, y = 7, face = Direction.SOUTH),
                Obstacle(id = 3, x = 12, y = 9, face = Direction.EAST),
                Obstacle(id = 4, x = 15, y = 15, face = Direction.SOUTH),
                Obstacle(id = 5, x = 15, y = 4, face = Direction.NORTH),
            ),
            result.obstacles,
        )
    }

    @Test
    fun `the comma before the face letter is optional - both formats give the same obstacle`() {
        assertEquals(parse("10,14l").obstacles, parse("10,14,l").obstacles)
        assertEquals(listOf(Obstacle(id = 1, x = 10, y = 14, face = Direction.WEST)), parse("10,14,l").obstacles)
    }

    @Test
    fun `both formats can be mixed in one list`() {
        val result = parse("5,13,l\n5,7d\n12,9,r\n")

        assertTrue(result.isPlaceable)
        assertEquals(
            listOf(Direction.WEST, Direction.SOUTH, Direction.EAST),
            result.obstacles.map { it.face },
        )
    }

    @Test
    fun `spaces, capital letters and CRLF line endings are tolerated`() {
        val result = parse(" 3 , 4 L \r\n6,2U\r\n")

        assertTrue(result.isPlaceable)
        assertEquals(Obstacle(id = 1, x = 3, y = 4, face = Direction.WEST), result.obstacles[0])
        assertEquals(Obstacle(id = 2, x = 6, y = 2, face = Direction.NORTH), result.obstacles[1])
    }

    @Test
    fun `blank lines are ignored and do not consume an obstacle number`() {
        val result = parse("\n3,4l\n\n   \n5,2r\n\n")

        assertTrue(result.isPlaceable)
        assertEquals(listOf(1, 2), result.obstacles.map { it.id })
        assertEquals(listOf(1, 2), result.lines.map { it.number })
    }

    @Test
    fun `empty text is not placeable`() {
        assertFalse(parse("").isPlaceable)
        assertFalse(parse("\n\n  \n").isPlaceable)
    }

    @Test
    fun `coordinates 0 and 19 are accepted but 20 and above are rejected`() {
        assertTrue(parse("0,0u\n19,19d").isPlaceable)

        val xTooBig = parse("20,3u")
        assertFalse(xTooBig.isPlaceable)
        assertEquals("x=20 is outside 0-19", xTooBig.lines[0].error)

        val yTooBig = parse("3,20u")
        assertEquals("y=20 is outside 0-19", yTooBig.lines[0].error)
    }

    @Test
    fun `negative or non-numeric coordinates are rejected`() {
        assertEquals("x and y must be numbers", parse("-1,3u").lines[0].error)
        assertEquals("x and y must be numbers", parse("a,3u").lines[0].error)
        assertEquals("x and y must be numbers", parse("3,,u").lines[0].error)
        assertEquals("x and y must be numbers", parse("99999999999,3u").lines[0].error)
    }

    @Test
    fun `a line that does not end in a face letter is rejected`() {
        assertEquals(needFace, parse("3,4").lines[0].error)
        assertEquals(needFace, parse("3").lines[0].error)
        assertEquals(needFace, parse("3,4,").lines[0].error)
        assertEquals(needFace, parse("10,14l5").lines[0].error)
    }

    @Test
    fun `a face letter without x,y in front of it is rejected`() {
        assertEquals("need x,y before the face letter", parse("l").lines[0].error)
        assertEquals("need x,y before the face letter", parse("3l").lines[0].error)
        assertEquals("need x,y before the face letter", parse("1014l").lines[0].error)
    }

    @Test
    fun `more than two numbers before the face letter is rejected`() {
        assertEquals("too many commas", parse("1,3,4l").lines[0].error)
        assertEquals("too many commas", parse("1,3,4,l").lines[0].error)
    }

    @Test
    fun `an unknown face letter is rejected`() {
        assertEquals(badFace, parse("3,4x").lines[0].error)
        assertEquals(badFace, parse("3,4,x").lines[0].error)
        assertEquals(badFace, parse("3,4n").lines[0].error)
        assertEquals(badFace, parse("3,4,left").lines[0].error)
    }

    @Test
    fun `two face letters in a row are rejected - the letter must be last and only one`() {
        assertEquals("only one face letter, at the end", parse("10,14ll").lines[0].error)
        assertEquals("only one face letter, at the end", parse("10,14,ll").lines[0].error)
        assertEquals("only one face letter, at the end", parse("10,14ud").lines[0].error)
    }

    @Test
    fun `two obstacles on the same cell are rejected and the later line is the one marked`() {
        val result = parse("5,13l\n5,13r")

        assertFalse(result.isPlaceable)
        assertNull(result.lines[0].error)
        assertEquals("same cell as obstacle 1", result.lines[1].error)
    }

    @Test
    fun `an obstacle on a blocked robot cell is rejected`() {
        val robot = RobotPose(x = 2, y = 1, direction = Direction.NORTH)
        val blocked = robotBlockedCells(robot)

        assertFalse(parse("2,2u", blocked).isPlaceable)
        assertEquals("on the robot start squares", parse("2,2u", blocked).lines[0].error)
        assertTrue(parse("10,10u", blocked).isPlaceable)
    }

    @Test
    fun `robot start squares for the default pose are x1-2 y1-3`() {
        val cells = robotBlockedCells(RobotPose(x = 2, y = 1, direction = Direction.NORTH))

        assertEquals(
            setOf(
                GridCell(1, 1), GridCell(2, 1),
                GridCell(1, 2), GridCell(2, 2),
                GridCell(1, 3), GridCell(2, 3),
            ),
            cells,
        )
    }

    @Test
    fun `eight obstacles are accepted and a ninth is rejected`() {
        val eight = (0 until 8).joinToString("\n") { "${it + 5},10u" }
        assertTrue(parse(eight).isPlaceable)

        val nine = (0 until 9).joinToString("\n") { "${it + 5},10u" }
        val result = parse(nine)
        assertFalse(result.isPlaceable)
        assertEquals("more than 8 obstacles", result.lines.last().error)
        assertEquals(1, result.errorLines.size)
    }

    @Test
    fun `every bad line is reported, not just the first`() {
        val result = parse("5,13l\n5,7x\n20,3u\n5,13r\n")

        assertEquals(listOf(2, 3, 4), result.errorLines.map { it.number })
        assertEquals(listOf(1), result.obstacles.map { it.id })
    }

    @Test
    fun `obstaclesToText writes the new format and round-trips through the parser`() {
        val obstacles = listOf(
            Obstacle(id = 2, x = 5, y = 7, face = Direction.SOUTH),
            Obstacle(id = 1, x = 5, y = 13, face = Direction.WEST),
        )

        val text = obstaclesToText(obstacles)

        assertEquals("5,13l\n5,7d\n", text)
        assertEquals(obstacles.sortedBy { it.id }, parse(text).obstacles)
    }

    @Test
    fun `obstaclesToText skips obstacles that have no face yet`() {
        val obstacles = listOf(
            Obstacle(id = 1, x = 5, y = 13, face = null),
            Obstacle(id = 2, x = 5, y = 7, face = Direction.EAST),
        )

        assertEquals("5,7r\n", obstaclesToText(obstacles))
    }
}
