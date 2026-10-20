package com.example.myapplication.arena.components

import androidx.compose.foundation.background
import androidx.compose.foundation.border
import androidx.compose.foundation.clickable
import androidx.compose.foundation.gestures.detectDragGestures
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.heightIn
import androidx.compose.foundation.layout.offset
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.size
import androidx.compose.foundation.layout.widthIn
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.input.pointer.pointerInput
import androidx.compose.runtime.setValue
import androidx.compose.ui.layout.onGloballyPositioned
import androidx.compose.ui.layout.positionInWindow
import androidx.compose.ui.text.TextStyle
import androidx.compose.ui.text.style.TextAlign
import androidx.compose.ui.text.style.TextOverflow
import androidx.compose.ui.text.font.FontFamily
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.tooling.preview.Preview
import androidx.compose.ui.unit.IntOffset
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import com.example.myapplication.arena.ArenaColors
import com.example.myapplication.arena.ArenaConfig
import com.example.myapplication.arena.Direction
import com.example.myapplication.arena.EditMode
import com.example.myapplication.ui.theme.AppTheme
import com.example.myapplication.ui.theme.AppType
import com.example.myapplication.ui.theme.Dimens
import com.example.myapplication.ui.theme.MyApplicationTheme
import kotlin.math.roundToInt

/** Edit/Submit/Unlock control + obstacle-creation box. Fills the 40% row reserved beside
 * LinkBar — see the team design notes. */
@Composable
fun ObstacleCreationBar(
    editMode: EditMode,
    isPlanSent: Boolean,
    isReady: Boolean,
    isStartSent: Boolean,
    onToggleClick: () -> Unit,
    onStartClick: () -> Unit,
    onClearGridClick: () -> Unit,
    modifier: Modifier = Modifier,
    onTypeObstaclesClick: () -> Unit = {},
) {
    Row(
        modifier = modifier
            .heightIn(min = 72.dp)
            .background(MaterialTheme.colorScheme.surface, RoundedCornerShape(Dimens.RadiusMedium))
            .border(Dimens.BorderWidth, MaterialTheme.colorScheme.outline, RoundedCornerShape(Dimens.RadiusMedium))
            .padding(horizontal = Dimens.Space16),
        horizontalArrangement = Arrangement.spacedBy(Dimens.Space8),
        verticalAlignment = Alignment.CenterVertically,
    ) {
        EditSubmitButton(editMode = editMode, onClick = onToggleClick)

        if (editMode == EditMode.LOCKED && isPlanSent) {
            StartButton(enabled = isReady && !isStartSent, onClick = onStartClick)
        }

        if (editMode == EditMode.EDITING) {
            TypeObstaclesButton(onClick = onTypeObstaclesClick)
            ClearGridButton(onClick = onClearGridClick)
        }
    }
}

@Composable
private fun EditSubmitButton(editMode: EditMode, onClick: () -> Unit) {
    val colors = AppTheme.colors
    val (label, fill) = when (editMode) {
        EditMode.EDITING -> "Submit" to colors.linkOk
        EditMode.LOCKED -> "Edit Grid" to colors.linkIdle
    }
    Box(
        modifier = Modifier
            .height(Dimens.TouchTargetMin)
            .clip(RoundedCornerShape(Dimens.RadiusPill))
            .background(fill)
            .clickable(onClick = onClick)
            .padding(horizontal = Dimens.Space16),
        contentAlignment = Alignment.Center,
    ) {
        Text(text = label, style = AppType.Label, color = ArenaColors.SurfaceWhite)
    }
}

@Composable
private fun StartButton(enabled: Boolean, onClick: () -> Unit) {
    val colors = AppTheme.colors
    val fill = if (enabled) colors.linkOk else colors.linkIdle
    val alpha = if (enabled) 1f else 0.5f
    Box(
        modifier = Modifier
            .height(Dimens.TouchTargetMin)
            .clip(RoundedCornerShape(Dimens.RadiusPill))
            .background(fill.copy(alpha = alpha))
            .clickable(enabled = enabled, onClick = onClick)
            .padding(horizontal = Dimens.Space16),
        contentAlignment = Alignment.Center,
    ) {
        Text(text = "Start", style = AppType.Label, color = ArenaColors.SurfaceWhite.copy(alpha = alpha))
    }
}

private val TypeObstaclesButtonMaxWidth = 84.dp

/** Opens the "Type obstacles" popup (see TypeObstaclesDialog). Edit mode only. */
@Composable
private fun TypeObstaclesButton(onClick: () -> Unit) {
    Box(
        modifier = Modifier
            .heightIn(min = Dimens.TouchTargetMin)
            .widthIn(max = TypeObstaclesButtonMaxWidth)
            .clip(RoundedCornerShape(Dimens.RadiusPill))
            .background(ArenaColors.AccentIndigo)
            .clickable(onClick = onClick)
            .padding(horizontal = Dimens.Space8, vertical = Dimens.Space4),
        contentAlignment = Alignment.Center,
    ) {
        Text(
            text = "Type obstacles",
            style = AppType.Label,
            color = ArenaColors.SurfaceWhite,
            textAlign = TextAlign.Center,
            maxLines = 2,
        )
    }
}

private val ClearGridButtonMaxWidth = 76.dp

@Composable
private fun ClearGridButton(onClick: () -> Unit) {
    val colors = AppTheme.colors
    Box(
        modifier = Modifier
            .heightIn(min = Dimens.TouchTargetMin)
            .widthIn(max = ClearGridButtonMaxWidth)
            .clip(RoundedCornerShape(Dimens.RadiusPill))
            .background(colors.linkLost)
            .clickable(onClick = onClick)
            .padding(horizontal = Dimens.Space8, vertical = Dimens.Space4),
        contentAlignment = Alignment.Center,
    ) {
        Text(
            text = "Reset Grid",
            style = AppType.Label,
            color = ArenaColors.SurfaceWhite,
            textAlign = TextAlign.Center,
            maxLines = 2,
        )
    }
}

@Preview(name = "ObstacleCreationBar - states, Light", widthDp = 320)
@Composable
private fun ObstacleCreationBarLightPreview() {
    MyApplicationTheme(darkTheme = false) {
        Column(verticalArrangement = Arrangement.spacedBy(Dimens.Space16)) {
            ObstacleCreationBar(editMode = EditMode.LOCKED, isPlanSent = false, isReady = false, isStartSent = false, onToggleClick = {}, onStartClick = {}, onClearGridClick = {})
            ObstacleCreationBar(editMode = EditMode.LOCKED, isPlanSent = true, isReady = false, isStartSent = false, onToggleClick = {}, onStartClick = {}, onClearGridClick = {})
            ObstacleCreationBar(editMode = EditMode.LOCKED, isPlanSent = true, isReady = true, isStartSent = false, onToggleClick = {}, onStartClick = {}, onClearGridClick = {})
            ObstacleCreationBar(editMode = EditMode.EDITING, isPlanSent = false, isReady = false, isStartSent = false, onToggleClick = {}, onStartClick = {}, onClearGridClick = {})
        }
    }
}

@Preview(name = "ObstacleCreationBar - states, Dark", widthDp = 320)
@Composable
private fun ObstacleCreationBarDarkPreview() {
    MyApplicationTheme(darkTheme = true) {
        Column(verticalArrangement = Arrangement.spacedBy(Dimens.Space16)) {
            ObstacleCreationBar(editMode = EditMode.LOCKED, isPlanSent = false, isReady = false, isStartSent = false, onToggleClick = {}, onStartClick = {}, onClearGridClick = {})
            ObstacleCreationBar(editMode = EditMode.LOCKED, isPlanSent = true, isReady = false, isStartSent = false, onToggleClick = {}, onStartClick = {}, onClearGridClick = {})
            ObstacleCreationBar(editMode = EditMode.LOCKED, isPlanSent = true, isReady = true, isStartSent = false, onToggleClick = {}, onStartClick = {}, onClearGridClick = {})
            ObstacleCreationBar(editMode = EditMode.EDITING, isPlanSent = false, isReady = false, isStartSent = false, onToggleClick = {}, onStartClick = {}, onClearGridClick = {})
        }
    }
}
