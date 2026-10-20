package com.ntu.mdp.t7.feature.task1_arena.components

import androidx.compose.foundation.background
import androidx.compose.foundation.border
import androidx.compose.foundation.clickable
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.text.style.TextOverflow
import androidx.compose.ui.tooling.preview.Preview
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import com.ntu.mdp.t7.core.model.EditMode
import com.ntu.mdp.t7.core.model.Obstacle
import com.ntu.mdp.t7.core.model.ObstaclePreset
import com.ntu.mdp.t7.ui.theme.AppTheme
import com.ntu.mdp.t7.ui.theme.AppType
import com.ntu.mdp.t7.ui.theme.Dimens
import com.ntu.mdp.t7.ui.theme.MyApplicationTheme

@Composable
fun PresetPanel(
    presets: List<ObstaclePreset?>,
    currentObstacles: List<Obstacle>,
    editMode: EditMode,
    isPagerLocked: Boolean,
    onPagerLockToggle: () -> Unit,
    onSavePreset: (slotIndex: Int) -> Unit,
    onLoadPreset: (slotIndex: Int) -> Unit,
    onClearPreset: (slotIndex: Int) -> Unit,
    modifier: Modifier = Modifier,
) {
    CollapsibleSidePanel(
        title = "Test Presets",
        isPagerLocked = isPagerLocked,
        onPagerLockToggle = onPagerLockToggle,
        modifier = modifier
    ) {
        Column(verticalArrangement = Arrangement.spacedBy(Dimens.Space8)) {
            for (slotIndex in 0..2) {
                val preset = presets.getOrNull(slotIndex)
                PresetSlotRow(
                    slotIndex = slotIndex,
                    preset = preset,
                    hasCurrentObstacles = currentObstacles.isNotEmpty(),
                    isEditing = editMode == EditMode.EDITING,
                    onSave = { onSavePreset(slotIndex) },
                    onLoad = { onLoadPreset(slotIndex) },
                    onClear = { onClearPreset(slotIndex) },
                )
            }
        }
    }
}

@Composable
private fun PresetSlotRow(
    slotIndex: Int,
    preset: ObstaclePreset?,
    hasCurrentObstacles: Boolean,
    isEditing: Boolean,
    onSave: () -> Unit,
    onLoad: () -> Unit,
    onClear: () -> Unit,
) {
    val slotTitle = "Preset ${slotIndex + 1}"
    val statusText = if (preset != null) "${preset.obstacles.size} obstacles" else "Empty"

    Row(
        modifier = Modifier
            .fillMaxWidth()
            .clip(RoundedCornerShape(Dimens.RadiusMedium))
            .background(MaterialTheme.colorScheme.surfaceVariant.copy(alpha = 0.5f))
            .padding(horizontal = Dimens.Space12, vertical = Dimens.Space8),
        horizontalArrangement = Arrangement.SpaceBetween,
        verticalAlignment = Alignment.CenterVertically,
    ) {
        Column(modifier = Modifier.weight(1f)) {
            Text(
                text = slotTitle,
                style = AppType.Label,
                color = MaterialTheme.colorScheme.onSurface,
                fontWeight = FontWeight.SemiBold,
                maxLines = 1,
                overflow = TextOverflow.Ellipsis,
            )
            Text(
                text = statusText,
                style = AppType.Body,
                fontSize = 11.sp,
                color = MaterialTheme.colorScheme.onSurfaceVariant,
                maxLines = 1,
            )
        }

        Row(horizontalArrangement = Arrangement.spacedBy(Dimens.Space4)) {
            ActionButton(
                text = "Save",
                enabled = hasCurrentObstacles,
                isPrimary = false,
                onClick = onSave,
            )

            ActionButton(
                text = "Load",
                enabled = preset != null && isEditing,
                isPrimary = true,
                onClick = onLoad,
            )

            if (preset != null) {
                ActionButton(
                    text = "✕",
                    enabled = true,
                    isPrimary = false,
                    isDestructive = true,
                    onClick = onClear,
                )
            }
        }
    }
}

@Composable
private fun ActionButton(
    text: String,
    enabled: Boolean,
    isPrimary: Boolean,
    isDestructive: Boolean = false,
    onClick: () -> Unit,
) {
    val colors = AppTheme.colors
    val backgroundColor = when {
        !enabled -> MaterialTheme.colorScheme.outline.copy(alpha = 0.2f)
        isDestructive -> colors.linkLost.copy(alpha = 0.2f)
        isPrimary -> MaterialTheme.colorScheme.primary
        else -> MaterialTheme.colorScheme.surface
    }

    val textColor = when {
        !enabled -> MaterialTheme.colorScheme.onSurfaceVariant.copy(alpha = 0.4f)
        isDestructive -> colors.linkLost
        isPrimary -> MaterialTheme.colorScheme.onPrimary
        else -> MaterialTheme.colorScheme.onSurface
    }

    Box(
        modifier = Modifier
            .height(28.dp)
            .clip(RoundedCornerShape(6.dp))
            .background(backgroundColor)
            .then(
                if (!isPrimary && enabled && !isDestructive) {
                    Modifier.border(1.dp, MaterialTheme.colorScheme.outline, RoundedCornerShape(6.dp))
                } else {
                    Modifier
                },
            )
            .then(if (enabled) Modifier.clickable(onClick = onClick) else Modifier)
            .padding(horizontal = 8.dp),
        contentAlignment = Alignment.Center,
    ) {
        Text(
            text = text,
            style = AppType.Label,
            fontSize = 11.sp,
            color = textColor,
            fontWeight = FontWeight.Medium,
        )
    }
}

@Preview(name = "PresetPanel - Light", widthDp = 300)
@Composable
private fun PresetPanelLightPreview() {
    MyApplicationTheme(darkTheme = false) {
        PresetPanel(
            presets = listOf(
                ObstaclePreset(0, listOf(Obstacle(1, 5, 5))),
                null,
                null,
            ),
            currentObstacles = listOf(Obstacle(1, 5, 5)),
            editMode = EditMode.EDITING,
            isPagerLocked = true,
            onPagerLockToggle = {},
            onSavePreset = {},
            onLoadPreset = {},
            onClearPreset = {},
        )
    }
}
