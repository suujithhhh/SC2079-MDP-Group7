package com.ntu.mdp.t7.feature.task1_arena

import com.ntu.mdp.t7.core.model.Direction
import com.ntu.mdp.t7.core.model.EditMode
import com.ntu.mdp.t7.core.model.Obstacle
import com.ntu.mdp.t7.core.model.ObstaclePreset
import com.ntu.mdp.t7.core.model.RobotPose

data class Task1UiState(
    val robot: RobotPose = RobotPose(x = 2, y = 1, direction = Direction.NORTH),
    val obstacles: List<Obstacle> = emptyList(),
    val editMode: EditMode = EditMode.LOCKED,
    val isKeypadDialogOpen: Boolean = false,
    val presets: List<ObstaclePreset?> = listOf(null, null),
    val isNewPlacement: Boolean = false,
    val obstacleFaceDialogTarget: Int? = null,
    val obstacleIdDraft: String = "",
    val obstacleFaceDraft: Direction? = null,
    val obstacleIdError: String? = null,
    val robotDirectionPickerVisible: Boolean = false,
    val isPlanSent: Boolean = false,
    val isReady: Boolean = false,
    val isStartSent: Boolean = false,
    val isPagerLocked: Boolean = true,
    val highlightedColumn: Int? = null,
    val highlightedRow: Int? = null,
)
