package com.ntu.mdp.t7.core.protocol

import com.ntu.mdp.t7.core.model.Direction
import com.ntu.mdp.t7.core.model.Obstacle
import com.ntu.mdp.t7.core.model.RobotPose
import com.ntu.mdp.t7.core.model.TurnType
import org.junit.Assert.assertEquals
import org.junit.Test

class ProtocolSerializerTest {

    @Test
    fun `formats ADD with B-prefixed id`() {
        assertEquals("ADD,B1,(10,6)", ProtocolSerializer.addObstacle(obstacleId = 1, x = 10, y = 6))
    }

    @Test
    fun `formats SUB with B-prefixed id`() {
        assertEquals("SUB,B1", ProtocolSerializer.removeObstacle(obstacleId = 1))
    }

    @Test
    fun `formats FACE with B-prefixed id and direction code`() {
        assertEquals("FACE,B2,N", ProtocolSerializer.face(obstacleId = 2, direction = Direction.NORTH))
    }

    @Test
    fun `formats forward and backward MOVE`() {
        assertEquals("MOVE,F", ProtocolSerializer.move(forward = true))
        assertEquals("MOVE,B", ProtocolSerializer.move(forward = false))
    }

    @Test
    fun `formats all 4 turn types`() {
        assertEquals("TURN,FL", ProtocolSerializer.turn(TurnType.FRONT_LEFT))
        assertEquals("TURN,FR", ProtocolSerializer.turn(TurnType.FRONT_RIGHT))
        assertEquals("TURN,BL", ProtocolSerializer.turn(TurnType.BACK_LEFT))
        assertEquals("TURN,BR", ProtocolSerializer.turn(TurnType.BACK_RIGHT))
    }

    @Test
    fun `formats PLAN_REQUEST with start pose and obstacle list`() {
        val robot = RobotPose(x = 2, y = 2, direction = Direction.NORTH)
        val obstacles = listOf(
            Obstacle(id = 1, x = 6, y = 10, face = Direction.NORTH),
            Obstacle(id = 2, x = 13, y = 5, face = Direction.WEST),
        )
        val expected = "{\"type\":\"PLAN_REQUEST\",\"protocol_version\":1,\"mode\":\"B3\"," +
            "\"coordinate_units\":\"grid\",\"start\":{\"x\":2,\"y\":2,\"heading\":\"N\"}," +
            "\"obstacles\":[{\"id\":1,\"x\":6,\"y\":10,\"face\":\"N\"}," +
            "{\"id\":2,\"x\":13,\"y\":5,\"face\":\"W\"}]}"
        assertEquals(expected, ProtocolSerializer.planRequest(robot, obstacles))
    }

    @Test
    fun `formats PLAN_REQUEST with an empty obstacle list`() {
        val robot = RobotPose(x = 0, y = 0, direction = Direction.SOUTH)
        val expected = "{\"type\":\"PLAN_REQUEST\",\"protocol_version\":1,\"mode\":\"B3\"," +
            "\"coordinate_units\":\"grid\",\"start\":{\"x\":0,\"y\":0,\"heading\":\"S\"},\"obstacles\":[]}"
        assertEquals(expected, ProtocolSerializer.planRequest(robot, emptyList()))
    }

    @Test(expected = IllegalArgumentException::class)
    fun `planRequest rejects an obstacle with no face`() {
        ProtocolSerializer.planRequest(
            RobotPose(x = 0, y = 0, direction = Direction.NORTH),
            listOf(Obstacle(id = 1, x = 5, y = 5)),
        )
    }
}
