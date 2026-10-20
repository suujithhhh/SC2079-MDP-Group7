package com.ntu.mdp.t7.feature.task1_arena.components

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
import com.ntu.mdp.t7.core.model.Direction
import com.ntu.mdp.t7.feature.task1_arena.ArenaColors
import com.ntu.mdp.t7.ui.theme.AppTheme
import com.ntu.mdp.t7.ui.theme.AppType
import com.ntu.mdp.t7.ui.theme.Dimens
import com.ntu.mdp.t7.ui.theme.MyApplicationTheme

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
        Surface(
            shape = RoundedCornerShape(Dimens.RadiusMedium),
            color = MaterialTheme.colorScheme.surface,
            tonalElevation = 6.dp,
        ) {
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
        modifier = Modifier.padding(Dimens.Space16),
        horizontalAlignment = Alignment.CenterHorizontally,
        verticalArrangement = Arrangement.spacedBy(Dimens.Space12),
    ) {
        if (isNewPlacement) {
            Column(horizontalAlignment = Alignment.CenterHorizontally) {
                Text(
                    text = "New Obstacle $obstacleId ($x, $y)",
                    style = AppType.Subtitle,
                    color = MaterialTheme.colorScheme.onSurface,
                )
                Text(
                    text = "(Pick Target Face)",
                    style = AppType.Label,
                    color = MaterialTheme.colorScheme.onSurfaceVariant,
                )
            }
        } else {
            Text(
                text = "Obstacle $obstacleId ($x, $y) — Target Face",
                style = AppType.Subtitle,
                color = MaterialTheme.colorScheme.onSurface,
            )
        }

        Compass(
            currentFace = currentFace,
            buttonSize = 44.dp,
            onFaceSelected = {
                onFaceSelected(it)
                onConfirm()
            }
        )

        if (!isNewPlacement) {
            Row(horizontalArrangement = Arrangement.spacedBy(Dimens.Space16)) {
                DeleteButton(modifier = Modifier.width(compassExtent(44.dp)), onClick = onDelete)
            }
        }
    }
}

@Composable
private fun DeleteButton(modifier: Modifier = Modifier, onClick: () -> Unit) {
    Box(
        modifier = modifier
            .height(34.dp)
            .background(AppTheme.colors.linkLost, RoundedCornerShape(Dimens.RadiusSmall))
            .clickable(onClick = onClick),
        contentAlignment = Alignment.Center,
    ) {
        Text(text = "Delete", style = AppType.Label, color = ArenaColors.SurfaceWhite)
    }
}

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
