package com.ntu.mdp.t7.core.model

/** Cardinal facing, matching the wire protocol's N/S/E/W codes (docs/design/protocol-spec.md). */
enum class Direction(val code: Char) {
    NORTH('N'),
    SOUTH('S'),
    EAST('E'),
    WEST('W');

    companion object {
        fun fromCode(code: Char): Direction? =
            entries.firstOrNull { it.code == code.uppercaseChar() }
    }
}

private val CLOCKWISE = listOf(Direction.NORTH, Direction.EAST, Direction.SOUTH, Direction.WEST)

fun Direction.rotatedClockwise(): Direction = CLOCKWISE[(CLOCKWISE.indexOf(this) + 1) % 4]
fun Direction.rotatedCounterClockwise(): Direction = CLOCKWISE[(CLOCKWISE.indexOf(this) + 3) % 4]

/** Grid delta for one forward step while facing this direction (y increases upward). */
fun Direction.forwardDelta(): Pair<Int, Int> = when (this) {
    Direction.NORTH -> 0 to 1
    Direction.SOUTH -> 0 to -1
    Direction.EAST -> 1 to 0
    Direction.WEST -> -1 to 0
}

/** The four L-shaped turn maneuvers replacing pure in-place rotation — see
 * the team design notes. */
enum class TurnType(val code: String) {
    FRONT_LEFT("FL"),
    FRONT_RIGHT("FR"),
    BACK_LEFT("BL"),
    BACK_RIGHT("BR"),
}

/**
 * Robot pose. (x, y) is the bottom-left cell of the robot's footprint bounding
 * box, not its centre — the chassis is a 3x3-cell square, so the box does not
 * change shape with heading.
 */
data class RobotPose(
    val x: Int,
    val y: Int,
    val direction: Direction,
)

/**
 * An obstacle on the arena grid. [targetId] and [face] are null until a TARGET
 * message is received for this obstacle.
 */
data class Obstacle(
    val id: Int,
    val x: Int,
    val y: Int,
    val targetId: Int? = null,
    val face: Direction? = null,
    val isSelected: Boolean = false,
)

/** A saved template configuration of obstacles for a preset slot (0-indexed: 0, 1, 2). */
data class ObstaclePreset(
    val slotIndex: Int,
    val obstacles: List<Obstacle>,
)

/** Edit-mode state machine for the obstacle creation bar: `EDITING` (button: Submit;
 * obstacles draggable/re-facable) -> `LOCKED` (button: Unlock). Starts at `LOCKED`. */
enum class EditMode { EDITING, LOCKED }

/** Assigns the smallest positive id not already used by [existingIds]. */
fun nextObstacleId(existingIds: Collection<Int>): Int {
    var candidate = 1
    while (candidate in existingIds) candidate++
    return candidate
}

/** Obstacles with no face set — Submit refuses the EDITING -> LOCKED transition while this is non-empty. */
fun obstaclesMissingFaces(obstacles: List<Obstacle>): List<Obstacle> = obstacles.filter { it.face == null }

/** Three severities, per docs/design/design-system-brief.md section 3: "info · event · alert". */
enum class StatusLevel { INFO, EVENT, ALERT }

data class StatusMessage(
    val time: String,
    val message: String,
    val level: StatusLevel = StatusLevel.INFO,
)

/**
 * Bluetooth link state, per docs/design/design-system-brief.md section 3: "idle ·
 * scanning · connecting · connected · lost (was connected, retrying)".
 */
sealed interface LinkState {
    data object Idle : LinkState
    data object Scanning : LinkState
    data class Connecting(val deviceName: String, val address: String) : LinkState
    data class Connected(val deviceName: String, val address: String, val signal: SignalQuality) : LinkState
    data class Lost(val deviceName: String, val address: String) : LinkState
}

/** A device shown in the picker sheet — paired or discovered. */
data class DiscoveredDevice(
    val name: String,
    val address: String,
    val signalStrength: Int? = null,
    val isPaired: Boolean = false,
)

/** Connection signal quality, driving the LinkBar's yellow/green distinction. */
enum class SignalQuality { STRONG, WEAK }

/** RSSI threshold (dBm) at/above which a link counts as strong. */
private const val STRONG_SIGNAL_THRESHOLD_DBM = -60

fun signalQualityFor(rssiDbm: Int?): SignalQuality =
    if (rssiDbm != null && rssiDbm >= STRONG_SIGNAL_THRESHOLD_DBM) SignalQuality.STRONG else SignalQuality.WEAK

/** Grid dimensions and cell metrics shared by geometry conversion and rendering. */
object ArenaConfig {
    const val GRID_SIZE = 20
    const val CELL_SIZE_DP = 36
    const val OBSTACLE_SIZE_DP = 32
    const val ROBOT_FOOTPRINT_LENGTH = 3
    const val ROBOT_FOOTPRINT_WIDTH = 2
    const val ROBOT_TURN_RADIUS_CELLS = 3
}

/** Used for Task 2's dynamically detected arrows. */
enum class ArrowDir { LEFT, RIGHT }

/** State for the "Scan for devices" bottom sheet. */
data class DevicePickerUiState(
    val isVisible: Boolean = false,
    val isScanning: Boolean = false,
    val devices: List<DiscoveredDevice> = emptyList(),
)
