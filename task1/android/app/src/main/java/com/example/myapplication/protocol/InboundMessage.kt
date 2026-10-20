package com.example.myapplication.protocol

import com.example.myapplication.arena.Direction

/** Parsed form of one already-line-framed message received from the robot/AMD tool. */
sealed interface InboundMessage {
    data class RobotPose(val x: Int, val y: Int, val direction: Direction) : InboundMessage
    data class TargetFound(val obstacleId: Int, val targetId: Int, val face: Direction?) : InboundMessage
    /** A structurally valid TARGET line whose targetId isn't in the approved image id set. */
    data class TargetRejected(val obstacleId: Int, val targetId: Int) : InboundMessage
    data class StatusMessage(val text: String) : InboundMessage
    object Ready : InboundMessage
}
