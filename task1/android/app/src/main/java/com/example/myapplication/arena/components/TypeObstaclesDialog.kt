package com.example.myapplication.arena.components

import androidx.compose.foundation.background
import androidx.compose.foundation.border
import androidx.compose.foundation.clickable
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.size
import androidx.compose.foundation.layout.widthIn
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.foundation.verticalScroll
import androidx.compose.material3.Surface
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.alpha
import androidx.compose.ui.draw.clip
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.text.TextStyle
import androidx.compose.ui.text.font.FontFamily
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.text.style.TextAlign
import androidx.compose.ui.tooling.preview.Preview
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import androidx.compose.ui.window.Dialog
import androidx.compose.ui.window.DialogProperties
import com.example.myapplication.arena.ArenaColors
import com.example.myapplication.arena.ArenaConfig
import com.example.myapplication.arena.Direction
import com.example.myapplication.arena.MAX_TEXT_OBSTACLES
import com.example.myapplication.arena.ObstacleTextParse
import com.example.myapplication.arena.RobotPose
import com.example.myapplication.arena.parseObstacleText
import com.example.myapplication.arena.robotBlockedCells
import com.example.myapplication.ui.theme.AppType
import com.example.myapplication.ui.theme.Dimens

/** Everything the "Type obstacles" button and popup can do, bundled so the screen only
 * threads one parameter. The defaults are no-ops, for previews. */
data class TypeObstaclesActions(
    val onOpen: () -> Unit = {},
    val onChar: (Char) -> Unit = {},
    val onBackspace: () -> Unit = {},
    val onClear: () -> Unit = {},
    val onPlace: () -> Unit = {},
    val onDismiss: () -> Unit = {},
)

private val KeyWidth = 76.dp
private val KeyHeight = 62.dp
private val Mono = FontFamily.Monospace

/**
 * Popup for entering the whole obstacle list as text, `x,y,face` per line (e.g. `5,13,l`),
 * with an on-screen keypad so the phone keyboard never opens. Line 1 is obstacle 1, line 2 is
 * obstacle 2, and so on. Each line shows what it was understood as, or why it was rejected;
 * Place stays disabled until every line is valid. See [parseObstacleText].
 */
@Composable
fun TypeObstaclesDialog(
    draft: String,
    robot: RobotPose,
    actions: TypeObstaclesActions,
) {
    Dialog(
        onDismissRequest = actions.onDismiss,
        properties = DialogProperties(usePlatformDefaultWidth = false),
    ) {
        Surface(
            shape = RoundedCornerShape(Dimens.RadiusLarge),
            color = ArenaColors.SurfaceWhite,
            modifier = Modifier.widthIn(max = 640.dp).fillMaxWidth(0.94f),
        ) {
            TypeObstaclesContent(draft = draft, robot = robot, actions = actions)
        }
    }
}

@Composable
fun TypeObstaclesContent(
    draft: String,
    robot: RobotPose,
    actions: TypeObstaclesActions,
) {
    val parsed = parseObstacleText(draft, ArenaConfig.GRID_SIZE, robotBlockedCells(robot))

    Column(
        modifier = Modifier.padding(Dimens.Space24),
        verticalArrangement = Arrangement.spacedBy(14.dp),
    ) {
        Column(verticalArrangement = Arrangement.spacedBy(Dimens.Space4)) {
            Text("Type obstacles", style = AppType.Title, color = ArenaColors.TextPrimary)
            Text(
                text = "Type x, comma, y, then a face letter, like 5,13l. The face letter ends the " +
                    "obstacle and starts the next line by itself. The first line is obstacle 1, the " +
                    "second is obstacle 2, and so on. Placing replaces everything on the grid.",
                style = AppType.Body,
                color = ArenaColors.TextSecondary,
            )
        }

        Editor(draft = draft, parsed = parsed)
        Summary(parsed = parsed, draft = draft)
        Keypad(actions = actions)

        Row(
            modifier = Modifier.fillMaxWidth(),
            horizontalArrangement = Arrangement.spacedBy(Dimens.Space12, Alignment.End),
        ) {
            ActionPill(label = "Cancel", fill = Color(0xFF6B7280), enabled = true, onClick = actions.onDismiss)
            ActionPill(label = "Place", fill = ArenaColors.AccentIndigo, enabled = parsed.isPlaceable, onClick = actions.onPlace)
        }
    }
}

@Composable
private fun Editor(draft: String, parsed: ObstacleTextParse) {
    val scroll = rememberScrollState()
    LaunchedEffect(draft) { scroll.scrollTo(scroll.maxValue) }

    // After a newline (or when empty) the cursor sits on a fresh, numberless line.
    val cursorOnFreshLine = draft.isEmpty() || draft.endsWith("\n")
    val lastNumber = parsed.lines.lastOrNull()?.number

    Column(
        modifier = Modifier
            .fillMaxWidth()
            .height(206.dp)
            .clip(RoundedCornerShape(Dimens.RadiusMedium))
            .background(ArenaColors.PageBackground)
            .border(Dimens.BorderWidth, ArenaColors.Border, RoundedCornerShape(Dimens.RadiusMedium))
            .verticalScroll(scroll)
            .padding(horizontal = 12.dp, vertical = 10.dp),
        verticalArrangement = Arrangement.spacedBy(Dimens.Space4),
    ) {
        parsed.lines.forEach { line ->
            val showCursor = !cursorOnFreshLine && line.number == lastNumber
            // The line still being typed (no face letter yet) isn't an error yet: keep it neutral.
            val inProgress = showCursor && line.error != null
            Row(
                modifier = Modifier.fillMaxWidth().height(34.dp),
                verticalAlignment = Alignment.CenterVertically,
                horizontalArrangement = Arrangement.spacedBy(10.dp),
            ) {
                Badge(number = line.number, active = true)
                Text(
                    text = line.text,
                    style = TextStyle(
                        fontFamily = Mono,
                        fontSize = 22.sp,
                        fontWeight = FontWeight.Medium,
                        color = if (line.error != null && !inProgress) ArenaColors.ObstacleUnrecognized else ArenaColors.TextPrimary,
                    ),
                )
                if (showCursor) Cursor()
                Spacer(Modifier.weight(1f))
                val obstacle = line.obstacle
                Text(
                    text = when {
                        obstacle != null -> "(${obstacle.x}, ${obstacle.y}) → face ${obstacle.face?.code}"
                        inProgress -> "…end with u, d, l or r"
                        else -> line.error.orEmpty()
                    },
                    style = TextStyle(
                        fontFamily = Mono,
                        fontSize = 14.sp,
                        fontWeight = FontWeight.Medium,
                        color = when {
                            obstacle != null -> ArenaColors.AccentGreen
                            inProgress -> ArenaColors.TextSecondary
                            else -> ArenaColors.ObstacleUnrecognized
                        },
                        textAlign = TextAlign.End,
                    ),
                )
            }
        }
        if (cursorOnFreshLine) {
            Row(
                modifier = Modifier.fillMaxWidth().height(34.dp),
                verticalAlignment = Alignment.CenterVertically,
                horizontalArrangement = Arrangement.spacedBy(10.dp),
            ) {
                Badge(number = parsed.lines.size + 1, active = false)
                Cursor()
                if (draft.isEmpty()) {
                    Text(
                        text = "e.g. 5,13l",
                        style = TextStyle(fontFamily = Mono, fontSize = 17.sp, color = Color(0xFF9AA3B4)),
                    )
                }
            }
        }
    }
}

@Composable
private fun Badge(number: Int, active: Boolean) {
    Box(
        modifier = Modifier
            .size(28.dp)
            .clip(RoundedCornerShape(6.dp))
            .background(if (active) ArenaColors.ObstacleUnrecognized else ArenaColors.GridLineMajor),
        contentAlignment = Alignment.Center,
    ) {
        Text(
            text = number.toString(),
            style = TextStyle(fontFamily = Mono, fontSize = 15.sp, fontWeight = FontWeight.Bold, color = ArenaColors.SurfaceWhite),
        )
    }
}

@Composable
private fun Cursor() {
    Box(Modifier.size(width = 2.dp, height = 24.dp).background(ArenaColors.AccentIndigo))
}

@Composable
private fun Summary(parsed: ObstacleTextParse, draft: String) {
    // The last line is still being typed when the draft doesn't end in a line break.
    val typing = parsed.lines.lastOrNull()?.takeIf { draft.isNotEmpty() && !draft.endsWith("\n") && it.error != null }
    val brokenLines = parsed.errorLines.filter { it != typing }
    val (text, color) = when {
        parsed.lines.isEmpty() ->
            "Face letter ends each line: u = up (N), d = down (S), l = left (W), r = right (E)" to ArenaColors.TextSecondary
        brokenLines.isNotEmpty() ->
            "Fix line ${brokenLines.joinToString(", ") { it.number.toString() }} before placing. " +
                "Nothing is placed until every line is OK." to ArenaColors.ObstacleUnrecognized
        typing != null ->
            "Finish line ${typing.number} with a face letter: u, d, l or r." to ArenaColors.TextSecondary
        else -> {
            val n = parsed.obstacles.size
            "$n obstacle${if (n == 1) "" else "s"} ready (max $MAX_TEXT_OBSTACLES). Place replaces the grid." to ArenaColors.AccentGreen
        }
    }
    Text(
        text = text,
        style = TextStyle(fontFamily = Mono, fontSize = 14.sp, fontWeight = FontWeight.Medium, color = color),
        modifier = Modifier.height(40.dp), // fixed so a wrapped second line doesn't resize the dialog
    )
}

@Composable
private fun Keypad(actions: TypeObstaclesActions) {
    Row(
        modifier = Modifier.fillMaxWidth(),
        horizontalArrangement = Arrangement.spacedBy(24.dp, Alignment.CenterHorizontally),
    ) {
        Column(verticalArrangement = Arrangement.spacedBy(Dimens.Space8)) {
            Row(horizontalArrangement = Arrangement.spacedBy(Dimens.Space8)) { listOf('7', '8', '9').forEach { CharKey(it, actions) } }
            Row(horizontalArrangement = Arrangement.spacedBy(Dimens.Space8)) { listOf('4', '5', '6').forEach { CharKey(it, actions) } }
            Row(horizontalArrangement = Arrangement.spacedBy(Dimens.Space8)) { listOf('1', '2', '3').forEach { CharKey(it, actions) } }
            Row(horizontalArrangement = Arrangement.spacedBy(Dimens.Space8)) {
                CharKey(',', actions)
                CharKey('0', actions)
                PadKey(label = "⌫", container = ArenaColors.PageBackground, onClick = actions.onBackspace)
            }
        }
        Column(verticalArrangement = Arrangement.spacedBy(Dimens.Space8)) {
            Row(horizontalArrangement = Arrangement.spacedBy(Dimens.Space8)) {
                KeySpacer()
                FaceKey('u', Direction.NORTH, actions)
                KeySpacer()
            }
            Row(horizontalArrangement = Arrangement.spacedBy(Dimens.Space8)) {
                FaceKey('l', Direction.WEST, actions)
                KeySpacer()
                FaceKey('r', Direction.EAST, actions)
            }
            Row(horizontalArrangement = Arrangement.spacedBy(Dimens.Space8)) {
                KeySpacer()
                FaceKey('d', Direction.SOUTH, actions)
                KeySpacer()
            }
            Row(horizontalArrangement = Arrangement.spacedBy(Dimens.Space8)) {
                KeySpacer()
                PadKey(label = "Clear", container = ArenaColors.PageBackground, labelSize = 18.sp, onClick = actions.onClear)
                KeySpacer()
            }
        }
    }
}

@Composable
private fun CharKey(char: Char, actions: TypeObstaclesActions) {
    PadKey(label = char.toString(), container = ArenaColors.SurfaceWhite, onClick = { actions.onChar(char) })
}

@Composable
private fun FaceKey(letter: Char, face: Direction, actions: TypeObstaclesActions) {
    PadKey(
        label = letter.uppercaseChar().toString(),
        hint = "→ ${face.code}",
        container = ArenaColors.ActiveBackground,
        labelColor = Color(0xFF2C3AA8),
        onClick = { actions.onChar(letter) },
    )
}

@Composable
private fun KeySpacer() {
    Spacer(Modifier.size(width = KeyWidth, height = KeyHeight))
}

@Composable
private fun PadKey(
    label: String,
    container: Color,
    onClick: () -> Unit,
    hint: String? = null,
    labelColor: Color = ArenaColors.TextPrimary,
    labelSize: androidx.compose.ui.unit.TextUnit = 26.sp,
) {
    Column(
        modifier = Modifier
            .size(width = KeyWidth, height = KeyHeight)
            .clip(RoundedCornerShape(Dimens.RadiusMedium))
            .background(container)
            .border(Dimens.BorderWidth, ArenaColors.Border, RoundedCornerShape(Dimens.RadiusMedium))
            .clickable(onClick = onClick),
        horizontalAlignment = Alignment.CenterHorizontally,
        verticalArrangement = Arrangement.Center,
    ) {
        Text(text = label, style = TextStyle(fontFamily = Mono, fontSize = labelSize, fontWeight = FontWeight.Medium, color = labelColor))
        if (hint != null) {
            Text(text = hint, style = TextStyle(fontSize = 11.sp, fontWeight = FontWeight.Medium, color = ArenaColors.TextSecondary))
        }
    }
}

@Composable
private fun ActionPill(label: String, fill: Color, enabled: Boolean, onClick: () -> Unit) {
    Box(
        modifier = Modifier
            .height(52.dp)
            .alpha(if (enabled) 1f else 0.4f)
            .clip(RoundedCornerShape(Dimens.RadiusPill))
            .background(fill)
            .clickable(enabled = enabled, onClick = onClick)
            .padding(horizontal = 28.dp),
        contentAlignment = Alignment.Center,
    ) {
        Text(text = label, style = AppType.Subtitle, color = ArenaColors.SurfaceWhite)
    }
}

@Preview(name = "TypeObstaclesDialog - task sheet typed", widthDp = 640)
@Composable
private fun TypeObstaclesPreview() {
    Surface(color = ArenaColors.SurfaceWhite) {
        TypeObstaclesContent(
            draft = "5,13l\n5,7d\n12,9r\n15,15d\n15,4u\n",
            robot = RobotPose(x = 2, y = 1, direction = Direction.NORTH),
            actions = TypeObstaclesActions(),
        )
    }
}

@Preview(name = "TypeObstaclesDialog - errors", widthDp = 640)
@Composable
private fun TypeObstaclesErrorsPreview() {
    Surface(color = ArenaColors.SurfaceWhite) {
        TypeObstaclesContent(
            draft = "5,13l\n5,7x\n20,3u\n5,13r\n2,2u",
            robot = RobotPose(x = 2, y = 1, direction = Direction.NORTH),
            actions = TypeObstaclesActions(),
        )
    }
}
