package com.ntu.mdp.t7.feature.task2_pathing.components

import androidx.compose.foundation.LocalIndication
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
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.rememberCoroutineScope
import androidx.compose.runtime.rememberUpdatedState
import androidx.compose.runtime.setValue
import androidx.compose.foundation.gestures.detectTapGestures
import androidx.compose.foundation.indication
import androidx.compose.foundation.interaction.MutableInteractionSource
import androidx.compose.foundation.interaction.PressInteraction
import androidx.compose.ui.input.pointer.pointerInput
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.sp
import kotlinx.coroutines.delay
import kotlinx.coroutines.launch
import androidx.compose.foundation.clickable
import androidx.compose.foundation.layout.Box
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.text.style.TextAlign
import androidx.compose.ui.unit.dp
import androidx.compose.foundation.layout.size
import androidx.compose.foundation.layout.widthIn
import androidx.compose.foundation.border
import com.ntu.mdp.t7.feature.connection.components.RunTimerCard
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
    elapsedMs: Long = 0L,
    isTimerRunning: Boolean = false,
) {
    CollapsibleSidePanel(
        title = "Task 2 Test",
        isPagerLocked = isPagerLocked,
        onPagerLockToggle = onPagerLockToggle,
        alwaysVisibleContent = {
            RunTimerCard(
                elapsedMs = elapsedMs,
                isRunning = isTimerRunning,
                modifier = Modifier.fillMaxWidth()
            )
        },
        modifier = modifier
    ) {
        Column(verticalArrangement = Arrangement.spacedBy(Dimens.Space8)) {
            Task2ParameterRow(
                label = "Distance 1",
                value = "$dist1",
                onDecrease = { if (dist1 > 60) onDist1Change(-10) },
                onIncrease = { if (dist1 < 150) onDist1Change(10) },
                decreaseEnabled = dist1 > 60,
                increaseEnabled = dist1 < 150
            )
            Task2ParameterRow(
                label = "Distance 2",
                value = "$dist2",
                onDecrease = { if (dist2 > 60) onDist2Change(-10) },
                onIncrease = { if (dist2 < 150) onDist2Change(10) },
                decreaseEnabled = dist2 > 60,
                increaseEnabled = dist2 < 150
            )
            Task2ParameterRow(
                label = "Obstacle 2",
                value = "${obs2Length}x10",
                onDecrease = { if (obs2Length > 30) onObs2LengthChange(-30) },
                onIncrease = { if (obs2Length < 90) onObs2LengthChange(30) },
                decreaseEnabled = obs2Length > 30,
                increaseEnabled = obs2Length < 90
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
    decreaseEnabled: Boolean = true,
    increaseEnabled: Boolean = true,
) {
    Row(
        modifier = Modifier
            .fillMaxWidth()
            .clip(RoundedCornerShape(Dimens.RadiusMedium))
            .background(MaterialTheme.colorScheme.surfaceVariant.copy(alpha = 0.4f))
            .padding(horizontal = Dimens.Space12, vertical = Dimens.Space8),
        horizontalArrangement = Arrangement.SpaceBetween,
        verticalAlignment = Alignment.CenterVertically
    ) {
        Text(
            text = label,
            style = AppType.Subtitle,
            color = MaterialTheme.colorScheme.onSurfaceVariant,
        )

        Row(
            verticalAlignment = Alignment.CenterVertically,
            modifier = Modifier
                .background(MaterialTheme.colorScheme.surface, RoundedCornerShape(Dimens.RadiusPill))
                .border(Dimens.BorderWidth, MaterialTheme.colorScheme.outlineVariant, RoundedCornerShape(Dimens.RadiusPill))
        ) {
            RepeatingStepperButton(
                text = "-",
                enabled = decreaseEnabled,
                onClick = onDecrease,
                modifier = Modifier
                    .size(48.dp)
                    .clip(RoundedCornerShape(topStart = Dimens.RadiusPill, bottomStart = Dimens.RadiusPill))
            )

            Text(
                text = value,
                style = AppType.Mono,
                fontSize = 16.sp,
                fontWeight = FontWeight.Bold,
                color = MaterialTheme.colorScheme.onSurface,
                modifier = Modifier.widthIn(min = 60.dp),
                textAlign = TextAlign.Center
            )

            RepeatingStepperButton(
                text = "+",
                enabled = increaseEnabled,
                onClick = onIncrease,
                modifier = Modifier
                    .size(48.dp)
                    .clip(RoundedCornerShape(topEnd = Dimens.RadiusPill, bottomEnd = Dimens.RadiusPill))
            )
        }
    }
}

@Composable
private fun RepeatingStepperButton(
    text: String,
    enabled: Boolean,
    onClick: () -> Unit,
    modifier: Modifier = Modifier
) {
    val interactionSource = remember { MutableInteractionSource() }
    val scope = rememberCoroutineScope()
    var isPressed by remember { mutableStateOf(false) }
    var currentPress by remember { mutableStateOf<PressInteraction.Press?>(null) }
    val currentOnClick by rememberUpdatedState(onClick)
    val currentEnabled by rememberUpdatedState(enabled)

    LaunchedEffect(isPressed) {
        if (isPressed && currentEnabled) {
            currentOnClick()
            delay(400) // Initial delay before repeating
            while (isPressed && currentEnabled) {
                delay(100) // Repeat interval
                currentOnClick()
            }
        }
    }

    // Force release state when disabled to clear any stuck ripples
    LaunchedEffect(currentEnabled) {
        if (!currentEnabled) {
            isPressed = false
            currentPress?.let { press ->
                interactionSource.emit(PressInteraction.Cancel(press))
                currentPress = null
            }
        }
    }

    Box(
        contentAlignment = Alignment.Center,
        modifier = modifier
            .background(if (currentEnabled) MaterialTheme.colorScheme.primaryContainer else MaterialTheme.colorScheme.surfaceVariant.copy(alpha = 0.5f))
            .pointerInput(currentEnabled) {
                if (!currentEnabled) return@pointerInput
                detectTapGestures(
                    onPress = { offset ->
                        val press = PressInteraction.Press(offset)
                        currentPress = press
                        scope.launch { interactionSource.emit(press) }
                        isPressed = true
                        
                        val success = tryAwaitRelease()
                        isPressed = false
                        
                        currentPress?.let { p ->
                            scope.launch {
                                interactionSource.emit(if (success) PressInteraction.Release(p) else PressInteraction.Cancel(p))
                            }
                            currentPress = null
                        }
                    }
                )
            }
            .indication(interactionSource, LocalIndication.current)
    ) {
        Text(
            text = text,
            style = AppType.Subtitle,
            fontSize = 24.sp,
            fontWeight = FontWeight.Medium,
            color = if (currentEnabled) MaterialTheme.colorScheme.primary else MaterialTheme.colorScheme.onSurface.copy(alpha = 0.38f),
            textAlign = TextAlign.Center
        )
    }
}
