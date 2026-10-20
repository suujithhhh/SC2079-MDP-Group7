package com.ntu.mdp.t7.feature.task2_pathing.components

import androidx.compose.foundation.background
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.text.style.TextAlign
import com.ntu.mdp.t7.feature.task1_arena.components.CollapsibleSidePanel
import com.ntu.mdp.t7.feature.task1_arena.components.PillButton
import com.ntu.mdp.t7.ui.theme.AppType
import com.ntu.mdp.t7.ui.theme.Dimens

@Composable
fun Task2SettingsPanel(
    dist1: Int,
    dist2: Int,
    obs2Length: Int,
    onDist1Change: (delta: Int) -> Unit,
    onDist2Change: (delta: Int) -> Unit,
    onObs2LengthChange: (delta: Int) -> Unit,
    isPagerLocked: Boolean,
    onPagerLockToggle: () -> Unit,
    modifier: Modifier = Modifier,
) {
    CollapsibleSidePanel(
        title = "Task 2 Test",
        isPagerLocked = isPagerLocked,
        onPagerLockToggle = onPagerLockToggle,
        modifier = modifier
    ) {
        Column(verticalArrangement = Arrangement.spacedBy(Dimens.Space8)) {
            Task2ParameterRow(
                label = "Distance 1",
                value = "$dist1 cm",
                onDecrease = { onDist1Change(-10) },
                onIncrease = { onDist1Change(10) }
            )
            Task2ParameterRow(
                label = "Distance 2",
                value = "$dist2 cm",
                onDecrease = { onDist2Change(-10) },
                onIncrease = { onDist2Change(10) }
            )
            Task2ParameterRow(
                label = "Obstacle 2",
                value = "${obs2Length}x10 cm",
                onDecrease = { onObs2LengthChange(-10) },
                onIncrease = { onObs2LengthChange(10) }
            )
        }
    }
}

@Composable
private fun Task2ParameterRow(
    label: String,
    value: String,
    onDecrease: () -> Unit,
    onIncrease: () -> Unit,
) {
    Column(
        modifier = Modifier
            .fillMaxWidth()
            .clip(RoundedCornerShape(Dimens.RadiusMedium))
            .background(MaterialTheme.colorScheme.surfaceVariant.copy(alpha = 0.5f))
            .padding(horizontal = Dimens.Space12, vertical = Dimens.Space8),
        verticalArrangement = Arrangement.spacedBy(Dimens.Space8)
    ) {
        Text(
            text = label,
            style = AppType.Subtitle,
            color = MaterialTheme.colorScheme.onSurfaceVariant,
        )
        Row(
            modifier = Modifier.fillMaxWidth(),
            horizontalArrangement = Arrangement.spacedBy(Dimens.Space8),
            verticalAlignment = Alignment.CenterVertically
        ) {
            PillButton(label = "-", onClick = onDecrease, modifier = Modifier.weight(1f))
            Text(
                text = value,
                style = AppType.Mono,
                color = MaterialTheme.colorScheme.onSurfaceVariant,
                modifier = Modifier.weight(1f),
                textAlign = TextAlign.Center
            )
            PillButton(label = "+", onClick = onIncrease, modifier = Modifier.weight(1f))
        }
    }
}
