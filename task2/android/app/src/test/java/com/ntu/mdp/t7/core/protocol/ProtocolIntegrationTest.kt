package com.ntu.mdp.t7.core.protocol

import com.ntu.mdp.t7.core.model.Direction
import org.junit.Assert.assertEquals
import org.junit.Test

class ProtocolIntegrationTest {

    @Test
    fun `two concatenated messages are framed then both parse correctly`() {
        val framer = LineFramer()
        val lines = framer.feed("ROBOT,7,2,N\nMSG,[Moving]\n")

        val messages = lines.map { ProtocolParser.parse(it) }

        assertEquals(
            listOf(
                InboundMessage.RobotPose(7, 2, Direction.NORTH),
                InboundMessage.StatusMessage("Moving"),
            ),
            messages,
        )
    }

    @Test
    fun `a message split across two reads is framed then parses correctly once complete`() {
        val framer = LineFramer()
        val firstBatch = framer.feed("TARGET,B2,")
        val secondBatch = framer.feed("11,N\n")

        assertEquals(emptyList<String>(), firstBatch)
        val messages = secondBatch.map { ProtocolParser.parse(it) }
        assertEquals(
            listOf(InboundMessage.TargetFound(obstacleId = 2, targetId = 11, face = Direction.NORTH)),
            messages,
        )
    }
}
