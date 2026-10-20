package com.example.myapplication.arena.components

import androidx.compose.foundation.background
import androidx.compose.foundation.clickable
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.width
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Surface
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.tooling.preview.Preview
import androidx.compose.ui.unit.dp
import androidx.compose.ui.window.Dialog
import com.example.myapplication.arena.ArenaColors
import com.example.myapplication.arena.Direction
import com.example.myapplication.ui.theme.AppTheme
import com.example.myapplication.ui.theme.AppType
import com.example.myapplication.ui.theme.Dimens
import com.example.myapplication.ui.theme.MyApplicationTheme

@Composable
fun ObstacleFaceDialog(
    obstacleId: Int,
    x: Int,
    y: Int,
    currentFace: Direction?,
    isNewPlacement: Boolean,
    onFaceSelected: (Direction) -> Unit,
    onDelete: () -> Unit,
    onConfirm: () -> Unit,
    onDismiss: () -> Unit,
) {
    Dialog(onDismissRequest = onDismiss) {
        Surface(shape = RoundedCornerShape(Dimens.RadiusLarge), color = MaterialTheme.colorScheme.surface) {
            ObstacleFaceDialogContent(
                obstacleId = obstacleId,
                x = x,
                y = y,
                currentFace = currentFace,
                isNewPlacement = isNewPlacement,
                onFaceSelected = onFaceSelected,
                onDelete = onDelete,
                onConfirm = onConfirm,
            )
        }
    }
}

@Composable
fun ObstacleFaceDialogContent(
    obstacleId: Int,
    x: Int,
    y: Int,
    currentFace: Direction?,
    isNewPlacement: Boolean,
    onFaceSelected: (Direction) -> Unit,
    onDelete: () -> Unit,
    onConfirm: () -> Unit,
) {
    Column(
        modifier = Modifier.padding(Dimens.Space24),
        horizontalAlignment = Alignment.CenterHorizontally,
        verticalArrangement = Arrangement.spacedBy(Dimens.Space16),
    ) {
        if (isNewPlacement) {
            Column(horizontalAlignment = Alignment.CenterHorizontally) {
                Text(
                    text = "New Obstacle $obstacleId ($x, $y)",
                    style = AppType.Title,
                    color = MaterialTheme.colorScheme.onSurface,
                )
                Text(
                    text = "(Pick a Target Face)",
                    style = AppType.Body,
                    color = MaterialTheme.colorScheme.onSurface,
                )
            }
        } else {
            Text(
                text = "Obstacle $obstacleId ($x, $y) — pick a target face",
                style = AppType.Title,
                color = MaterialTheme.colorScheme.onSurface,
            )
        }

        Compass(currentFace = currentFace, onFaceSelected = {
            onFaceSelected(it)
            onConfirm()
        })

        if (!isNewPlacement) {
            Row(horizontalArrangement = Arrangement.spacedBy(Dimens.Space16)) {
                DeleteButton(modifier = Modifier.width(CompassRowWidth), onClick = onDelete)
            }
        }
    }
}

@Composable
private fun DeleteButton(modifier: Modifier = Modifier, onClick: () -> Unit) {
    Box(
        modifier = modifier
            .height(Dimens.TouchTargetMin)
            .background(AppTheme.colors.linkLost, RoundedCornerShape(Dimens.RadiusMedium))
            .clickable(onClick = onClick),
        contentAlignment = Alignment.Center,
    ) {
        Text(text = "Delete", style = AppType.Label, color = ArenaColors.SurfaceWhite)
    }
}

private val CompassRowWidth = compassExtent()

@Preview(name = "ObstacleFaceDialog - new obstacle, no face yet", widthDp = 420)
@Composable
private fun ObstacleFaceDialogNewPreview() {
    MyApplicationTheme(darkTheme = false) {
        Surface(color = MaterialTheme.colorScheme.surface) {
            ObstacleFaceDialogContent(
                obstacleId = 7,
                x = 5,
                y = 10,
                currentFace = null,
                isNewPlacement = true,
                onFaceSelected = {},
                onDelete = {},
                onConfirm = {},
            )
        }
    }
}

@Preview(name = "ObstacleFaceDialog - existing obstacle, face selected", widthDp = 420)
@Composable
private fun ObstacleFaceDialogExistingPreview() {
    MyApplicationTheme(darkTheme = false) {
        Surface(color = MaterialTheme.colorScheme.surface) {
            ObstacleFaceDialogContent(
                obstacleId = 7,
                x = 5,
                y = 10,
                currentFace = Direction.NORTH,
                isNewPlacement = false,
                onFaceSelected = {},
                onDelete = {},
                onConfirm = {},
            )
        }
    }
}
