package com.example.myapplication.protocol

import com.example.myapplication.arena.Direction
import com.example.myapplication.arena.Obstacle
import com.example.myapplication.arena.RobotPose
import com.example.myapplication.arena.TurnType

/** Builds outbound wire messages per docs/design/protocol-spec.md. The transport layer adds the `\n` terminator. */
object ProtocolSerializer {
    fun addObstacle(obstacleId: Int, x: Int, y: Int): String = "ADD,B$obstacleId,($x,$y)"
    fun removeObstacle(obstacleId: Int): String = "SUB,B$obstacleId"
    fun face(obstacleId: Int, direction: Direction): String = "FACE,B$obstacleId,${direction.code}"
    fun move(forward: Boolean): String = "MOVE,${if (forward) "F" else "B"}"
    fun turn(type: TurnType): String = "TURN,${type.code}"
    fun start(): String = "START"

    /**
     * Builds the PLAN_REQUEST JSON sent to the algorithm team on Submit (see
     * the team design notes).
     * Every obstacle must have a face set — callers validate this before submitting,
     * since the algorithm team's planner needs a face per obstacle.
     *
     * Hand-built with string templates rather than org.json: Android's bundled
     * org.json.JSONObject throws "not mocked" in this project's JVM unit tests (no
     * Robolectric / testOptions.unitTests.isReturnDefaultValues configured), and the
     * payload's fields are all ints or single-letter direction codes, so no escaping
     * is needed.
     */
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
