package com.example.myapplication.protocol

import com.example.myapplication.arena.Direction
import org.junit.Assert.assertEquals
import org.junit.Assert.assertNull
import org.junit.Test

class ProtocolParserTest {

    @Test
    fun `parses ROBOT position update`() {
        val message = ProtocolParser.parse("ROBOT,7,2,N")
        assertEquals(InboundMessage.RobotPose(7, 2, Direction.NORTH), message)
    }

    @Test
    fun `parses ROBOT case-insensitively and trims whitespace`() {
        val message = ProtocolParser.parse("  robot , 7 , 2 , n ")
        assertEquals(InboundMessage.RobotPose(7, 2, Direction.NORTH), message)
    }

    @Test
    fun `parses two-arity TARGET`() {
        val message = ProtocolParser.parse("TARGET,B2,11")
        assertEquals(InboundMessage.TargetFound(obstacleId = 2, targetId = 11, face = null), message)
    }

    @Test
    fun `parses three-arity TARGET with face`() {
        val message = ProtocolParser.parse("TARGET,B2,11,N")
        assertEquals(InboundMessage.TargetFound(obstacleId = 2, targetId = 11, face = Direction.NORTH), message)
    }

    @Test
    fun `TARGET accepts a bare obstacle number without the B prefix`() {
        val message = ProtocolParser.parse("TARGET,2,11")
        assertEquals(InboundMessage.TargetFound(obstacleId = 2, targetId = 11, face = null), message)
    }

    @Test
    fun `TARGET with an image id outside the valid set is rejected, not dropped`() {
        assertEquals(
            InboundMessage.TargetRejected(obstacleId = 2, targetId = 99),
            ProtocolParser.parse("TARGET,B2,99"),
        )
    }

    @Test
    fun `TARGET rejects image id 1`() {
        assertEquals(
            InboundMessage.TargetRejected(obstacleId = 26, targetId = 1),
            ProtocolParser.parse("TARGET,B26,1"),
        )
    }

    @Test
    fun `TARGET accepts every value in the valid image id set`() {
        val validImageIds = listOf(
            11, 12, 13, 14, 15, 16, 17, 18, 19,
            36, 37, 38, 39, 40,
            20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35,
        )
        validImageIds.forEach { id ->
            assertEquals(
                InboundMessage.TargetFound(obstacleId = 2, targetId = id, face = null),
                ProtocolParser.parse("TARGET,B2,$id"),
            )
        }
    }

    @Test
    fun `parses MSG payload out of its brackets`() {
        val message = ProtocolParser.parse("MSG,[Looking for target 2]")
        assertEquals(InboundMessage.StatusMessage("Looking for target 2"), message)
    }

    @Test
    fun `unknown verb is dropped`() {
        assertNull(ProtocolParser.parse("PING,1"))
    }

    @Test
    fun `wrong arity is dropped`() {
        assertNull(ProtocolParser.parse("ROBOT,7,2"))
    }

    @Test
    fun `unparseable field is dropped`() {
        assertNull(ProtocolParser.parse("ROBOT,seven,2,N"))
    }

    @Test
    fun `invalid direction code is dropped`() {
        assertNull(ProtocolParser.parse("ROBOT,7,2,Q"))
    }

    @Test
    fun `blank line is dropped`() {
        assertNull(ProtocolParser.parse("   "))
    }

    @Test
    fun `parseObstacleId accepts B-prefixed and bare forms`() {
        assertEquals(2, ProtocolParser.parseObstacleId("B2"))
        assertEquals(2, ProtocolParser.parseObstacleId("b2"))
        assertEquals(2, ProtocolParser.parseObstacleId("2"))
        assertNull(ProtocolParser.parseObstacleId("BX"))
    }

    @Test
    fun `parses READY verb with no fields`() {
        assertEquals(InboundMessage.Ready, ProtocolParser.parse("READY"))
    }

    @Test
    fun `parses READY case-insensitively and trims whitespace`() {
        assertEquals(InboundMessage.Ready, ProtocolParser.parse("  ready  "))
    }
}
