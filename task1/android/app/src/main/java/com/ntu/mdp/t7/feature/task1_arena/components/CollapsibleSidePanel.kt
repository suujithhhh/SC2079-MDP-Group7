package com.ntu.mdp.t7.feature.task1_arena.components

import androidx.compose.animation.AnimatedVisibility
import androidx.compose.animation.expandVertically
import androidx.compose.animation.fadeIn
import androidx.compose.animation.fadeOut
import androidx.compose.animation.shrinkVertically
import androidx.compose.foundation.background
import androidx.compose.foundation.border
import androidx.compose.foundation.clickable
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Text
import androidx.compose.material3.TextButton
import androidx.compose.runtime.Composable
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.saveable.rememberSaveable
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.text.font.FontWeight
import com.ntu.mdp.t7.feature.task1_arena.ArenaColors
import com.ntu.mdp.t7.ui.theme.AppType
import com.ntu.mdp.t7.ui.theme.Dimens

@Composable
fun CollapsibleSidePanel(
    title: String,
    isPagerLocked: Boolean,
    onPagerLockToggle: () -> Unit,
    modifier: Modifier = Modifier,
    initiallyExpanded: Boolean = false,
    alwaysVisibleContent: (@Composable () -> Unit)? = null,
    content: @Composable () -> Unit,
) {
    var isExpanded by rememberSaveable { mutableStateOf(initiallyExpanded) }

    Column(
        modifier = modifier
            .background(MaterialTheme.colorScheme.surface, RoundedCornerShape(Dimens.RadiusMedium))
            .border(Dimens.BorderWidth, MaterialTheme.colorScheme.outline, RoundedCornerShape(Dimens.RadiusMedium))
            .padding(Dimens.Space12),
        verticalArrangement = Arrangement.spacedBy(Dimens.Space8),
    ) {
        Row(
            modifier = Modifier
                .fillMaxWidth()
                .clip(RoundedCornerShape(Dimens.RadiusSmall))
                .clickable { isExpanded = !isExpanded }
                .padding(vertical = Dimens.Space4, horizontal = Dimens.Space4),
            horizontalArrangement = Arrangement.SpaceBetween,
            verticalAlignment = Alignment.CenterVertically,
        ) {
            Text(
                text = title,
                style = AppType.Title,
                color = MaterialTheme.colorScheme.onSurface,
                fontWeight = FontWeight.Bold,
            )
            Text(
                text = if (isExpanded) "▲" else "▼",
                style = AppType.Title,
                color = MaterialTheme.colorScheme.onSurfaceVariant,
            )
        }

        alwaysVisibleContent?.invoke()

        AnimatedVisibility(
            visible = isExpanded,
            enter = expandVertically() + fadeIn(),
            exit = shrinkVertically() + fadeOut(),
        ) {
            content()
        }

        Row(
            modifier = Modifier.fillMaxWidth(),
            horizontalArrangement = Arrangement.End,
            verticalAlignment = Alignment.CenterVertically
        ) {
            TextButton(onClick = onPagerLockToggle) {
                Text(
                    text = if (isPagerLocked) "🔒 Locked" else "🔓 Unlocked",
                    style = AppType.Label,
                    color = if (isPagerLocked) ArenaColors.ObstacleUnrecognized else ArenaColors.TextSecondary
                )
            }
        }
    }
}
