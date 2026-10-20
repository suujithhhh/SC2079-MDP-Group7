package com.ntu.mdp.t7.core.protocol

import com.ntu.mdp.t7.core.model.Direction
import com.ntu.mdp.t7.core.model.Obstacle
import com.ntu.mdp.t7.core.model.RobotPose
import com.ntu.mdp.t7.core.model.TurnType

object ProtocolSerializer {
    fun addObstacle(obstacleId: Int, x: Int, y: Int): String = "ADD,B$obstacleId,($x,$y)"
    fun removeObstacle(obstacleId: Int): String = "SUB,B$obstacleId"
    fun face(obstacleId: Int, direction: Direction): String = "FACE,B$obstacleId,${direction.code}"
    fun move(forward: Boolean): String = "MOVE,${if (forward) "F" else "B"}"
    fun turn(type: TurnType): String = "TURN,${type.code}"
    fun start(): String = "START"

    fun planRequest(robot: RobotPose, obstacles: List<Obstacle>): String {
        val obstaclesJson = obstacles.joinToString(separator = ",") { obstacle ->
            val face = requireNotNull(obstacle.face) { "Obstacle ${obstacle.id} has no face set" }
            "{\"id\":${obstacle.id},\"x\":${obstacle.x},\"y\":${obstacle.y},\"face\":\"${face.code}\"}"
        }
        return "{\"type\":\"PLAN_REQUEST\",\"protocol_version\":1,\"mode\":\"B3\"," +
            "\"coordinate_units\":\"grid\",\"start\":{\"x\":${robot.x},\"y\":${robot.y}," +
            "\"heading\":\"${robot.direction.code}\"},\"obstacles\":[$obstaclesJson]}"
    }
}
