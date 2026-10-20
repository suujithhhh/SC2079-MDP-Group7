package com.example.myapplication.arena.components

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
import com.example.myapplication.arena.Direction
import com.example.myapplication.ui.theme.AppType
import com.example.myapplication.ui.theme.Dimens

/** Default button size, matching [ObstacleFaceDialog]'s original compass sizing. */
val DefaultCompassButtonSize = 64.dp

/** Total width/height of the plus-shaped [Compass] at a given [buttonSize] — 3 buttons
 * plus 2 inter-button gaps. Callers sizing a container around a [Compass] should use
 * this rather than restating the arithmetic themselves. */
fun compassExtent(buttonSize: Dp = DefaultCompassButtonSize): Dp = buttonSize * 3 + Dimens.Space4 * 2

/** Plus-shaped N/S/E/W face picker used by [ObstacleFaceDialog]. [currentFace] highlights the
 * matching button; tapping any button calls [onFaceSelected] with that direction — callers
 * decide whether that commits immediately or only selects a draft. [FaceButton] is also used
 * standalone (outside this cluster) by the robot direction picker's surrounding buttons — see
 * the team design notes. */
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
