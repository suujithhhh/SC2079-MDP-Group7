package com.example.myapplication.protocol

import com.example.myapplication.arena.Direction
import java.util.Locale

/**
 * Parses one already-line-framed wire message (see [LineFramer]). Never throws: malformed
 * or unknown input returns null so the caller can drop-and-log per protocol-spec.md's
 * parser rules, rather than crashing or surfacing the raw line to the user.
 */
object ProtocolParser {

    /** The 30 image IDs the recognition module can report, per the team's confirmed list.
     * Deliberately does not include 1 — see docs/design/protocol-spec.md's confirmed list. */
    private val VALID_IMAGE_IDS = setOf(
        11, 12, 13, 14, 15, 16, 17, 18, 19,
        36, 37, 38, 39, 40,
        20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35,
    )

    fun parse(rawLine: String): InboundMessage? {
        val trimmed = rawLine.trim()
        if (trimmed.isEmpty()) return null
        val commaIndex = trimmed.indexOf(',')
        val verb = (if (commaIndex == -1) trimmed else trimmed.substring(0, commaIndex)).trim().uppercase(Locale.ROOT)
        val remainder = if (commaIndex == -1) "" else trimmed.substring(commaIndex + 1)
        return when (verb) {
            "ROBOT" -> parseRobot(remainder)
            "TARGET" -> parseTarget(remainder)
            "MSG" -> parseMsg(remainder)
            "READY" -> InboundMessage.Ready
            else -> null
        }
    }

    /** Accepts "B2", "OBS2", "O2", or bare "2", per protocol-spec.md: "Be liberal on input." */
    fun parseObstacleId(field: String): Int? {
        val cleaned = field.trim()
            .removePrefix("B").removePrefix("b")
            .removePrefix("OBS").removePrefix("obs")
            .removePrefix("O").removePrefix("o")
            .filter { it.isDigit() }
        return cleaned.toIntOrNull()
    }

    private fun parseRobot(remainder: String): InboundMessage.RobotPose? {
        val fields = remainder.split(",").map { it.trim() }
        if (fields.size != 3) return null
        val x = fields[0].toIntOrNull() ?: return null
        val y = fields[1].toIntOrNull() ?: return null
        if (fields[2].isEmpty()) return null
        val direction = Direction.fromCode(fields[2][0]) ?: return null
        return InboundMessage.RobotPose(x, y, direction)
    }

    private fun parseTarget(remainder: String): InboundMessage? {
        val fields = remainder.split(",").map { it.trim() }
        if (fields.size < 2) return null
        val obstacleId = parseObstacleId(fields[0]) ?: return null
        val targetId = fields[1].filter { it.isDigit() }.toIntOrNull() ?: return null
        val face = if (fields.size >= 3 && fields[2].isNotBlank()) {
            val code = fields[2].trim().firstOrNull()
            if (code != null) Direction.fromCode(code) else null
        } else {
            null
        }
        if (targetId !in VALID_IMAGE_IDS) return InboundMessage.TargetRejected(obstacleId, targetId)
        return InboundMessage.TargetFound(obstacleId, targetId, face)
    }

    private fun parseMsg(remainder: String): InboundMessage.StatusMessage? {
        val trimmed = remainder.trim()
        if (!trimmed.startsWith("[") || !trimmed.endsWith("]")) return null
        return InboundMessage.StatusMessage(trimmed.substring(1, trimmed.length - 1))
    }
}
