package com.ntu.mdp.t7.feature.task1_arena.components

import androidx.compose.foundation.background
import androidx.compose.foundation.border
import androidx.compose.foundation.clickable
import androidx.compose.foundation.layout.*
import androidx.compose.foundation.lazy.LazyColumn
import androidx.compose.foundation.lazy.items
import androidx.compose.foundation.lazy.rememberLazyListState
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material3.*
import androidx.compose.runtime.*
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.text.style.TextAlign
import androidx.compose.ui.unit.Dp
import androidx.compose.ui.unit.dp
import androidx.compose.ui.window.Dialog
import androidx.compose.ui.window.DialogProperties
import com.ntu.mdp.t7.core.model.Direction
import com.ntu.mdp.t7.core.model.Obstacle
import com.ntu.mdp.t7.feature.task1_arena.ArenaColors
import com.ntu.mdp.t7.ui.theme.AppTheme
import com.ntu.mdp.t7.ui.theme.AppType
import com.ntu.mdp.t7.ui.theme.Dimens

private fun renumberObstacles(list: List<Obstacle>): List<Obstacle> {
    return list.mapIndexed { index, obs -> obs.copy(id = index + 1) }
}

private val NumpadRows = listOf(
    listOf("1", "2", "3"),
    listOf("4", "5", "6"),
    listOf("7", "8", "9"),
    listOf("C", "0", ","),
)

@Composable
fun ObstacleKeypadDialog(
    initialObstacles: List<Obstacle>,
    onDismiss: () -> Unit,
    onPlaceObstacles: (List<Obstacle>) -> Unit,
) {
    var draftObstacles by remember { mutableStateOf(renumberObstacles(initialObstacles)) }
    var currentInput by remember { mutableStateOf("") }
    var errorMessage by remember { mutableStateOf<String?>(null) }

    val commaCount = currentInput.count { it == ',' }
    val isCommaEnabled = (commaCount < 2) && currentInput.isNotEmpty() && currentInput.last().isDigit()
    val listState = rememberLazyListState()

    LaunchedEffect(draftObstacles.size) {
        if (draftObstacles.isNotEmpty()) {
            listState.animateScrollToItem(draftObstacles.size - 1)
        }
    }

    val onKeyPress: (String) -> Unit = remember(isCommaEnabled, commaCount, currentInput, draftObstacles) {
        { key ->
            errorMessage = null
            when (key) {
                "C", "Clear" -> {
                    currentInput = ""
                }
                "," -> {
                    if (isCommaEnabled) {
                        currentInput += ","
                    }
                }
                "N", "S", "E", "W" -> {
                    var rawInput = currentInput
                    val commas = rawInput.count { it == ',' }
                    if (commas == 0 && rawInput.isNotEmpty()) {
                        rawInput = "$rawInput,0,$key"
                    } else if (commas == 1 && rawInput.substringAfterLast(',').isNotEmpty()) {
                        rawInput = "$rawInput,$key"
                    } else if (commas == 1 && rawInput.endsWith(",")) {
                        rawInput = "${rawInput}0,$key"
                    } else if (commas == 2 && rawInput.endsWith(",")) {
                        rawInput = "$rawInput$key"
                    } else if (commas == 2 && rawInput.last().isLetter()) {
                        rawInput = rawInput.dropLast(1) + key
                    } else {
                        rawInput = "$rawInput,$key"
                    }

                    val parts = rawInput.split(",")
                    if (parts.size == 3 && parts[2].isNotEmpty()) {
                        val x = parts[0].toIntOrNull()
                        val y = parts[1].toIntOrNull()
                        val face = Direction.fromCode(parts[2][0])

                        if (x != null && y != null && face != null) {
                            if (x !in 0..19 || y !in 0..19) {
                                errorMessage = "Coordinates must be 0 - 19"
                                currentInput = rawInput
                            } else if (draftObstacles.any { it.x == x && it.y == y }) {
                                errorMessage = "Cell ($x, $y) is already occupied"
                                currentInput = rawInput
                            } else {
                                val newObstacle = Obstacle(id = draftObstacles.size + 1, x = x, y = y, face = face)
                                draftObstacles = renumberObstacles(draftObstacles + newObstacle)
                                currentInput = ""
                            }
                        } else {
                            currentInput = rawInput
                        }
                    } else {
                        currentInput = rawInput
                    }
                }
                "+ Add" -> {
                    val parts = currentInput.split(",")
                    if (parts.size == 3 && parts[2].isNotEmpty()) {
                        val x = parts[0].toIntOrNull()
                        val y = parts[1].toIntOrNull()
                        val face = Direction.fromCode(parts[2][0])

                        if (x != null && y != null && face != null) {
                            if (x !in 0..19 || y !in 0..19) {
                                errorMessage = "Coordinates must be 0 - 19"
                            } else if (draftObstacles.any { it.x == x && it.y == y }) {
                                errorMessage = "Cell ($x, $y) is already occupied"
                            } else {
                                val newObstacle = Obstacle(id = draftObstacles.size + 1, x = x, y = y, face = face)
                                draftObstacles = renumberObstacles(draftObstacles + newObstacle)
                                currentInput = ""
                            }
                        }
                    } else {
                        errorMessage = "Incomplete input. Format: X,Y,Face"
                    }
                }
                else -> {
                    if (commaCount < 2) {
                        val currentPart = currentInput.substringAfterLast(',')
                        if (currentPart.length < 2) {
                            val newInput = currentInput + key
                            val newPart = newInput.substringAfterLast(',')
                            if (newPart.length == 2) {
                                currentInput = "$newInput,"
                            } else {
                                currentInput = newInput
                            }
                        }
                    }
                }
            }
        }
    }

    Dialog(
        onDismissRequest = onDismiss,
        properties = DialogProperties(usePlatformDefaultWidth = false)
    ) {
        Surface(
            shape = RoundedCornerShape(Dimens.RadiusMedium),
            color = MaterialTheme.colorScheme.surface,
            tonalElevation = 8.dp,
            modifier = Modifier
                .fillMaxWidth(0.88f)
                .padding(Dimens.Space16)
        ) {
            Column(
                modifier = Modifier
                    .padding(Dimens.Space16)
                    .fillMaxWidth()
            ) {
                Text(
                    text = "Edit Obstacles",
                    style = AppType.Title,
                    color = ArenaColors.TextPrimary,
                    textAlign = TextAlign.Center,
                    modifier = Modifier
                        .fillMaxWidth()
                        .padding(bottom = Dimens.Space12)
                )

                if (draftObstacles.isNotEmpty()) {
                    Box(modifier = Modifier.weight(1f, fill = false).heightIn(max = 110.dp)) {
                        LazyColumn(
                            state = listState,
                            verticalArrangement = Arrangement.spacedBy(Dimens.Space8)
                        ) {
                            items(draftObstacles) { obs ->
                                Row(
                                    modifier = Modifier
                                        .fillMaxWidth()
                                        .background(ArenaColors.PanelBackground, RoundedCornerShape(Dimens.RadiusSmall))
                                        .padding(horizontal = Dimens.Space8, vertical = Dimens.Space4),
                                    verticalAlignment = Alignment.CenterVertically
                                ) {
                                    Text(
                                        text = "Obs ${obs.id}: (${obs.x}, ${obs.y}) ${obs.face?.code ?: ""}",
                                        style = AppType.Body,
                                        color = ArenaColors.TextPrimary,
                                        modifier = Modifier.weight(1f)
                                    )
                                    TextButton(
                                        onClick = {
                                            draftObstacles = renumberObstacles(draftObstacles.filter { it.id != obs.id })
                                        },
                                        contentPadding = PaddingValues(horizontal = Dimens.Space8)
                                    ) {
                                        Text("🗑 Del", color = ArenaColors.StatusErrorBorder)
                                    }
                                }
                            }
                        }
                    }
                    Spacer(modifier = Modifier.height(Dimens.Space12))
                }

                InputDisplayBox(input = currentInput, errorMessage = errorMessage)

                Spacer(modifier = Modifier.height(Dimens.Space16))

                Row(
                    modifier = Modifier.fillMaxWidth(),
                    horizontalArrangement = Arrangement.spacedBy(Dimens.Space24),
                    verticalAlignment = Alignment.Top
                ) {
                    Column(
                        modifier = Modifier.weight(1.1f),
                        verticalArrangement = Arrangement.spacedBy(Dimens.Space8)
                    ) {
                        Text("Coordinates (X, Y)", style = AppType.Label, color = ArenaColors.TextSecondary)
                        NumpadRows.forEach { row ->
                            Row(horizontalArrangement = Arrangement.spacedBy(Dimens.Space8)) {
                                row.forEach { key ->
                                    KeypadBtn(
                                        text = key,
                                        modifier = Modifier.weight(1f),
                                        enabled = if (key == ",") isCommaEnabled else true,
                                    ) { onKeyPress(key) }
                                }
                            }
                        }
                    }

                    Column(
                        modifier = Modifier.weight(1f),
                        horizontalAlignment = Alignment.CenterHorizontally,
                        verticalArrangement = Arrangement.spacedBy(Dimens.Space8)
                    ) {
                        Text("Face Direction", style = AppType.Label, color = ArenaColors.TextSecondary)

                        KeypadBtn(
                            text = "N",
                            height = 34.dp,
                            modifier = Modifier.width(42.dp),
                            onClick = { onKeyPress("N") }
                        )
                        Row(
                            horizontalArrangement = Arrangement.spacedBy(16.dp),
                            verticalAlignment = Alignment.CenterVertically
                        ) {
                            KeypadBtn(
                                text = "W",
                                height = 34.dp,
                                modifier = Modifier.width(42.dp),
                                onClick = { onKeyPress("W") }
                            )
                            KeypadBtn(
                                text = "E",
                                height = 34.dp,
                                modifier = Modifier.width(42.dp),
                                onClick = { onKeyPress("E") }
                            )
                        }
                        KeypadBtn(
                            text = "S",
                            height = 34.dp,
                            modifier = Modifier.width(42.dp),
                            onClick = { onKeyPress("S") }
                        )

                        Spacer(modifier = Modifier.height(Dimens.Space4))

                        Button(
                            onClick = { onKeyPress("+ Add") },
                            colors = ButtonDefaults.buttonColors(containerColor = AppTheme.colors.linkIdle),
                            contentPadding = PaddingValues(horizontal = Dimens.Space16, vertical = Dimens.Space8),
                            shape = RoundedCornerShape(Dimens.RadiusSmall),
                            modifier = Modifier.fillMaxWidth()
                        ) {
                            Text("+ Add Obstacle", style = AppType.Title, color = ArenaColors.SurfaceWhite)
                        }
                    }
                }

                Spacer(modifier = Modifier.height(24.dp))

                Button(
                    onClick = { onPlaceObstacles(draftObstacles) },
                    colors = ButtonDefaults.buttonColors(containerColor = AppTheme.colors.linkOk),
                    modifier = Modifier
                        .fillMaxWidth()
                        .height(44.dp)
                ) {
                    Text("Place Obstacles", style = AppType.Title, color = ArenaColors.SurfaceWhite)
                }
            }
        }
    }
}

@Composable
private fun InputDisplayBox(
    input: String,
    errorMessage: String?
) {
    Column(horizontalAlignment = Alignment.CenterHorizontally) {
        Box(
            modifier = Modifier
                .fillMaxWidth()
                .height(40.dp)
                .background(ArenaColors.PanelBackground, RoundedCornerShape(Dimens.RadiusSmall))
                .border(Dimens.BorderWidth, MaterialTheme.colorScheme.outline, RoundedCornerShape(Dimens.RadiusSmall)),
            contentAlignment = Alignment.Center
        ) {
            Text(
                text = input.ifEmpty { "Format: X, Y, Face" },
                style = AppType.Title,
                color = if (input.isEmpty()) ArenaColors.TextSecondary else ArenaColors.TextPrimary,
            )
        }

        if (errorMessage != null) {
            Text(
                text = errorMessage,
                color = ArenaColors.StatusErrorBorder,
                style = AppType.Label,
                modifier = Modifier.padding(top = Dimens.Space4)
            )
        }
    }
}

@Composable
private fun KeypadBtn(
    text: String,
    modifier: Modifier = Modifier,
    height: Dp = 38.dp,
    enabled: Boolean = true,
    onClick: () -> Unit
) {
    val alpha = if (enabled) 1f else 0.4f
    Box(
        modifier = modifier
            .height(height)
            .clip(RoundedCornerShape(Dimens.RadiusSmall))
            .background(MaterialTheme.colorScheme.surfaceVariant.copy(alpha = alpha))
            .clickable(enabled = enabled, onClick = onClick),
        contentAlignment = Alignment.Center
    ) {
        Text(
            text = text,
            style = AppType.Title,
            color = ArenaColors.TextPrimary.copy(alpha = alpha)
        )
    }
}
