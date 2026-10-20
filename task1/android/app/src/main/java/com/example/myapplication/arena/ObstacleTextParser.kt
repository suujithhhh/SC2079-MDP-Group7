package com.example.myapplication.arena

/** The planner takes at most this many obstacles (docs: PROTOCOL.md section 7, `POST /plan/v2`). */
const val MAX_TEXT_OBSTACLES = 8

/** Letters accepted in the face column of a typed obstacle line. */
val OBSTACLE_TEXT_FACE_LETTERS = charArrayOf('u', 'd', 'l', 'r')

/**
 * Maps the face letter typed by the user to the wire direction: the letter names the side of
 * the obstacle the image is on, as seen on the task sheet (up/down/left/right on the arena
 * map), so `u`=N, `d`=S, `l`=W, `r`=E. Case-insensitive; null for any other character.
 */
fun faceFromLetter(letter: Char): Direction? = when (letter.lowercaseChar()) {
    'u' -> Direction.NORTH
    'd' -> Direction.SOUTH
    'l' -> Direction.WEST
    'r' -> Direction.EAST
    else -> null
}

/** Inverse of [faceFromLetter]. */
fun faceToLetter(face: Direction): Char = when (face) {
    Direction.NORTH -> 'u'
    Direction.SOUTH -> 'd'
    Direction.WEST -> 'l'
    Direction.EAST -> 'r'
}

/**
 * One non-blank line of the typed list. [number] counts non-blank lines from 1 and doubles as
 * the obstacle id. Exactly one of [obstacle] and [error] is non-null.
 */
data class ObstacleTextLine(
    val number: Int,
    val text: String,
    val obstacle: Obstacle?,
    val error: String?,
)

/** Result of [parseObstacleText]. Nothing should be placed unless [isPlaceable]. */
data class ObstacleTextParse(val lines: List<ObstacleTextLine>) {
    val obstacles: List<Obstacle> get() = lines.mapNotNull { it.obstacle }
    val errorLines: List<ObstacleTextLine> get() = lines.filter { it.error != null }
    val isPlaceable: Boolean get() = lines.isNotEmpty() && errorLines.isEmpty()
}

/**
 * Parses the typed obstacle list: one obstacle per line, `x,y` followed by a face letter,
 * e.g. `5,13l`. The face letter ends the line; a comma before it (`5,13,l`) is accepted too.
 * Obstacle ids are assigned by order (first non-blank line is obstacle 1) so the user never
 * types them. Spaces and blank lines are ignored; the face letter is case-insensitive.
 *
 * Every line is checked, not just the first bad one, so the UI can mark them all. A line is
 * rejected when it doesn't end in a face letter, isn't `x,y` before that letter, x or y is
 * outside `0 until [gridSize]`, the letter isn't one of [OBSTACLE_TEXT_FACE_LETTERS], its cell
 * is in [blockedCells] (the robot's footprint), its cell repeats an earlier obstacle, or it is
 * past [MAX_TEXT_OBSTACLES].
 */
fun parseObstacleText(
    text: String,
    gridSize: Int,
    blockedCells: Set<GridCell>,
): ObstacleTextParse {
    val lines = mutableListOf<ObstacleTextLine>()
    val seen = mutableMapOf<GridCell, Int>()
    var number = 0
    for (rawLine in text.lineSequence()) {
        val compact = rawLine.filterNot { it.isWhitespace() }
        if (compact.isEmpty()) continue
        number++
        val (obstacle, error) = parseLine(number, compact, gridSize, blockedCells, seen)
        if (obstacle != null) seen[GridCell(obstacle.x, obstacle.y)] = number
        lines += ObstacleTextLine(number, compact, obstacle, error)
    }
    return ObstacleTextParse(lines)
}

private fun parseLine(
    number: Int,
    compact: String,
    gridSize: Int,
    blockedCells: Set<GridCell>,
    seen: Map<GridCell, Int>,
): Pair<Obstacle?, String?> {
    // The face letter ends the line, with or without a comma before it: 10,14l or 10,14,l.
    val faceChar = compact.last()
    if (!faceChar.isLetter()) return null to "end the line with a face letter: u, d, l or r"
    val face = faceFromLetter(faceChar) ?: return null to "face must be u, d, l or r"
    val coords = compact.dropLast(1).removeSuffix(",")
    if (coords.lastOrNull()?.isLetter() == true) return null to "only one face letter, at the end"
    val parts = coords.split(',')
    if (parts.size < 2) return null to "need x,y before the face letter"
    if (parts.size > 2) return null to "too many commas"
    val x = parts[0].toIntOrNull()?.takeIf { parts[0].all(Char::isDigit) }
    val y = parts[1].toIntOrNull()?.takeIf { parts[1].all(Char::isDigit) }
    if (x == null || y == null) return null to "x and y must be numbers"
    if (x !in 0 until gridSize) return null to "x=$x is outside 0-${gridSize - 1}"
    if (y !in 0 until gridSize) return null to "y=$y is outside 0-${gridSize - 1}"
    val cell = GridCell(x, y)
    if (cell in blockedCells) return null to "on the robot start squares"
    seen[cell]?.let { return null to "same cell as obstacle $it" }
    if (number > MAX_TEXT_OBSTACLES) return null to "more than $MAX_TEXT_OBSTACLES obstacles"
    return Obstacle(id = number, x = x, y = y, face = face) to null
}

/** The cells the robot occupies, so typed obstacles can't be placed on top of it. */
fun robotBlockedCells(robot: RobotPose): Set<GridCell> = robotFootprintCells(
    robot.x, robot.y, robot.direction,
    ArenaConfig.ROBOT_FOOTPRINT_LENGTH, ArenaConfig.ROBOT_FOOTPRINT_WIDTH,
)

/**
 * The list as typed text (one `x,yL` line per obstacle, trailing newline), for pre-filling
 * the popup from what's already on the grid. Obstacles without a face are skipped: the typed
 * format can't express them.
 */
fun obstaclesToText(obstacles: List<Obstacle>): String =
    obstacles.sortedBy { it.id }
        .filter { it.face != null }
        .joinToString(separator = "") { "${it.x},${it.y}${faceToLetter(it.face!!)}\n" }
