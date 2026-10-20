package com.ntu.mdp.t7.core.protocol

import com.ntu.mdp.t7.core.model.Direction

/** Parsed form of one already-line-framed message received from the robot/AMD tool. */
sealed interface InboundMessage {
    data class RobotPose(val x: Int, val y: Int, val direction: Direction) : InboundMessage
    /** Task 2: Continuous position in centimeters and heading in degrees (0-360). */
    data class RobotPoseDistance(val xCm: Float, val yCm: Float, val headingDeg: Float) : InboundMessage
    data class TargetFound(val obstacleId: Int, val targetId: Int, val face: Direction?, val imageUrl: String? = null) : InboundMessage
    /** A structurally valid TARGET line whose targetId isn't in the approved image id set. */
    data class TargetRejected(val obstacleId: Int, val targetId: Int) : InboundMessage
    data class StatusMessage(val text: String) : InboundMessage
    object Ready : InboundMessage
}
