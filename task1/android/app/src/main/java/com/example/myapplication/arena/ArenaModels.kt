package com.example.myapplication.arena

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
 * change shape with heading (see [com.example.myapplication.arena.footprintBox]).
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

/** Edit-mode state machine for the obstacle creation bar: `EDITING` (button: Submit;
 * obstacles draggable/re-facable) -> `LOCKED` (button: Unlock). [ArenaUiState] defaults to `LOCKED`; the app's
 * [ArenaViewModel] is created with `startEditing = true`, so the screen opens in `EDITING`. */
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

/** Connection signal quality, driving the LinkBar's yellow/green distinction. Real Classic
 * Bluetooth doesn't expose live RSSI post-connect — until the transport layer adds
 * heartbeat/latency-based detection, this is derived from the demo route's sample data. */
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

    /** The robot's physical turning radius, in cells — the lateral and
     * forward/backward displacement of an L-shaped turn (see [turnedPose]
     * in ArenaGeometry.kt and
     * the team design notes). Independent
     * of [ROBOT_FOOTPRINT_LENGTH]/[ROBOT_FOOTPRINT_WIDTH]. */
    const val ROBOT_TURN_RADIUS_CELLS = 3
}

data class ArenaUiState(
    val link: LinkState,
    val robot: RobotPose,
    val obstacles: List<Obstacle>,
    val statusLog: List<StatusMessage>,
    val editMode: EditMode = EditMode.LOCKED,
    /** The last successfully connected device, saved for Quick Connect. */
    val savedDevice: DiscoveredDevice? = null,
    /** True if the currently open dialog is for a new obstacle rather than editing an existing one. */
    val isNewPlacement: Boolean = false,
    /** Id of the obstacle currently being prompted for a face — shared by creation and
     * re-facing. Null = dialog hidden. */
    val obstacleFaceDialogTarget: Int? = null,
    /** Keypad-entered digits for a new obstacle's id, meaningful only while creating
     * (i.e. [obstacleFaceDialogTarget] refers to an id not yet in [obstacles]). Empty
     * means "auto-assign via [nextObstacleId] on confirm." */
    val obstacleIdDraft: String = "",
    /** Face selected (but not yet committed) for a new obstacle — null means none picked
     * yet, which keeps the Create button disabled. Not used when re-facing. */
    val obstacleFaceDraft: Direction? = null,
    /** Inline validation message for [obstacleIdDraft]/[obstacleFaceDraft] — e.g. a
     * duplicate id — shown under the keypad. Null = no error. */
    val obstacleIdError: String? = null,
    /** Raw wire strings from the most recent Submit, shown in [ProtocolLogDrawer]. Persists
     * across an Unlock; overwritten by the next Submit. */
    val rawProtocolLog: List<String> = emptyList(),
    /** True while the robot-direction picker (N/S/E/W buttons surrounding the robot) is
     * showing — set true by a successful
     * [com.example.myapplication.arena.ArenaViewModel.onRobotDragEnd] or by
     * [com.example.myapplication.arena.ArenaViewModel.onRobotTapped], cleared by picking a
     * direction ([com.example.myapplication.arena.ArenaViewModel.onRobotFaceSelected]) or by
     * tapping the robot again. Purely a UI visibility toggle — does not gate Submit (see
     * the team design notes). */
    val robotDirectionPickerVisible: Boolean = false,
    /** True if the user has submitted the current plan (EDITING -> LOCKED). Reset when unlocking.
     * Gates the visibility of the Start button. */
    val isPlanSent: Boolean = false,
    /** True when the Pi has explicitly signaled it is ready for a START command. */
    val isReady: Boolean = false,
    /** True if the user has tapped Start. Greyes out and disables the Start button. */
    val isStartSent: Boolean = false,
    /** The currently highlighted column, toggled by tapping a column label. Null if none. */
    val highlightedColumn: Int? = null,
    /** The currently highlighted row, toggled by tapping a row label. Null if none. */
    val highlightedRow: Int? = null,
    /** True while the "Type obstacles" popup is open (edit mode only). */
    val typeObstaclesVisible: Boolean = false,
    /** Text typed so far in the "Type obstacles" popup: one `x,y,face` per line (see
     * [parseObstacleText]). Meaningful only while [typeObstaclesVisible]. */
    val typeObstaclesDraft: String = "",
)

/** Starting state: empty grid, robot's camera at (2,1),
 * facing north — matches what "Clear Grid" restores the arena to. */
fun defaultArenaUiState(): ArenaUiState = ArenaUiState(
    link = LinkState.Idle,
    robot = RobotPose(x = 2, y = 1, direction = Direction.NORTH),
    obstacles = emptyList(),
    statusLog = emptyList(),
)

/** State for the "Scan for devices" bottom sheet. */
data class DevicePickerUiState(
    val isVisible: Boolean = false,
    val isScanning: Boolean = false,
    val devices: List<DiscoveredDevice> = emptyList(),
)
