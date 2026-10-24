package com.ntu.mdp.t7.feature.task1_arena.components

import androidx.compose.foundation.background
import androidx.compose.foundation.border
import androidx.compose.foundation.clickable
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.size
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.unit.Dp
import androidx.compose.ui.unit.dp
import com.ntu.mdp.t7.core.model.Direction
import com.ntu.mdp.t7.ui.theme.AppType
import com.ntu.mdp.t7.ui.theme.Dimens

val DefaultCompassButtonSize = 64.dp

fun compassExtent(buttonSize: Dp = DefaultCompassButtonSize): Dp = buttonSize * 3 + Dimens.Space4 * 2

@Composable
fun Compass(
    currentFace: Direction?,
    onFaceSelected: (Direction) -> Unit,
    buttonSize: Dp = DefaultCompassButtonSize,
) {
    Column(verticalArrangement = Arrangement.spacedBy(Dimens.Space4)) {
        Row(horizontalArrangement = Arrangement.spacedBy(Dimens.Space4)) {
            CompassSpacer(buttonSize)
            FaceButton(Direction.NORTH, currentFace, onFaceSelected, buttonSize)
            CompassSpacer(buttonSize)
        }
        Row(horizontalArrangement = Arrangement.spacedBy(Dimens.Space4)) {
            FaceButton(Direction.WEST, currentFace, onFaceSelected, buttonSize)
            CompassSpacer(buttonSize)
            FaceButton(Direction.EAST, currentFace, onFaceSelected, buttonSize)
        }
        Row(horizontalArrangement = Arrangement.spacedBy(Dimens.Space4)) {
            CompassSpacer(buttonSize)
            FaceButton(Direction.SOUTH, currentFace, onFaceSelected, buttonSize)
            CompassSpacer(buttonSize)
        }
    }
}

@Composable
private fun CompassSpacer(buttonSize: Dp) {
    Box(modifier = Modifier.size(buttonSize))
}

@Composable
fun FaceButton(
    direction: Direction,
    currentFace: Direction?,
    onFaceSelected: (Direction) -> Unit,
    buttonSize: Dp,
) {
    val selected = direction == currentFace
    Box(
        modifier = Modifier
            .size(buttonSize)
            .background(
                if (selected) MaterialTheme.colorScheme.primaryContainer else MaterialTheme.colorScheme.surfaceVariant,
                RoundedCornerShape(Dimens.RadiusMedium),
            )
            .border(Dimens.BorderWidth, MaterialTheme.colorScheme.outline, RoundedCornerShape(Dimens.RadiusMedium))
            .clickable { onFaceSelected(direction) },
        contentAlignment = Alignment.Center,
    ) {
        Text(text = direction.code.toString(), style = AppType.Title, color = MaterialTheme.colorScheme.onSurface)
    }
}
