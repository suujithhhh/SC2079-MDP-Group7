package com.ntu.mdp.t7.feature.task1_arena.components

import androidx.compose.foundation.background
import androidx.compose.foundation.border
import androidx.compose.foundation.clickable
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.heightIn
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.text.TextStyle
import androidx.compose.ui.text.font.FontFamily
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.text.style.TextAlign
import androidx.compose.ui.tooling.preview.Preview
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import com.ntu.mdp.t7.core.model.EditMode
import com.ntu.mdp.t7.feature.task1_arena.ArenaColors
import com.ntu.mdp.t7.ui.theme.AppTheme
import com.ntu.mdp.t7.ui.theme.AppType
import com.ntu.mdp.t7.ui.theme.Dimens
import com.ntu.mdp.t7.ui.theme.MyApplicationTheme

@Composable
fun ObstacleCreationBar(
    editMode: EditMode,
    isPlanSent: Boolean,
    isReady: Boolean,
    isStartSent: Boolean,
    onToggleClick: () -> Unit,
    onKeypadClick: () -> Unit,
    onStartClick: () -> Unit,
    onClearGridClick: () -> Unit,
    modifier: Modifier = Modifier,
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
        if (editMode == EditMode.LOCKED) {
            EditSubmitButton(label = "Edit Grid", fill = AppTheme.colors.linkIdle, onClick = onKeypadClick)

            if (isPlanSent) {
                StartButton(enabled = isReady && !isStartSent, onClick = onStartClick)
            }
        } else {
            EditSubmitButton(label = "Edit Grid", fill = AppTheme.colors.linkIdle, onClick = onKeypadClick)
            EditSubmitButton(label = "Submit", fill = AppTheme.colors.linkOk, onClick = onToggleClick)
            Spacer(modifier = Modifier.weight(1f))
            ClearGridButton(onClick = onClearGridClick)
        }
    }
}

@Composable
private fun EditSubmitButton(label: String, fill: Color, onClick: () -> Unit) {
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

@Composable
private fun ClearGridButton(onClick: () -> Unit) {
    val colors = AppTheme.colors
    Box(
        modifier = Modifier
            .height(30.dp)
            .clip(RoundedCornerShape(Dimens.RadiusPill))
            .background(colors.linkLost.copy(alpha = 0.85f))
            .clickable(onClick = onClick)
            .padding(horizontal = 10.dp, vertical = 2.dp),
        contentAlignment = Alignment.Center,
    ) {
        Text(
            text = "Reset Grid",
            style = TextStyle(
                fontFamily = FontFamily.Default,
                fontWeight = FontWeight.Medium,
                fontSize = 11.sp,
                color = ArenaColors.SurfaceWhite,
            ),
            textAlign = TextAlign.Center,
            maxLines = 1,
        )
    }
}

@Preview(name = "ObstacleCreationBar - states, Light", widthDp = 320)
@Composable
private fun ObstacleCreationBarLightPreview() {
    MyApplicationTheme(darkTheme = false) {
        Column(verticalArrangement = Arrangement.spacedBy(Dimens.Space16)) {
            ObstacleCreationBar(editMode = EditMode.LOCKED, isPlanSent = false, isReady = false, isStartSent = false, onToggleClick = {}, onKeypadClick = {}, onStartClick = {}, onClearGridClick = {})
            ObstacleCreationBar(editMode = EditMode.LOCKED, isPlanSent = true, isReady = false, isStartSent = false, onToggleClick = {}, onKeypadClick = {}, onStartClick = {}, onClearGridClick = {})
            ObstacleCreationBar(editMode = EditMode.LOCKED, isPlanSent = true, isReady = true, isStartSent = false, onToggleClick = {}, onKeypadClick = {}, onStartClick = {}, onClearGridClick = {})
            ObstacleCreationBar(editMode = EditMode.EDITING, isPlanSent = false, isReady = false, isStartSent = false, onToggleClick = {}, onKeypadClick = {}, onStartClick = {}, onClearGridClick = {})
        }
    }
}
