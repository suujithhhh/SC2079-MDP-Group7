package com.ntu.mdp.t7.feature.task2_pathing

import androidx.lifecycle.ViewModel
import com.ntu.mdp.t7.core.model.ArrowDir
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.flow.update

class Task2ViewModel : ViewModel() {

    private val _uiState = MutableStateFlow(Task2UiState())
    val uiState: StateFlow<Task2UiState> = _uiState.asStateFlow()

    fun onTask2Distance1Change(delta: Int) {
        _uiState.update { it.copy(task2Distance1 = (it.task2Distance1 + delta).coerceIn(60, 150)) }
    }

    fun onTask2Distance2Change(delta: Int) {
        _uiState.update { it.copy(task2Distance2 = (it.task2Distance2 + delta).coerceIn(60, 150)) }
    }

    fun onTask2Obs2LengthChange(delta: Int) {
        _uiState.update { it.copy(task2Obs2Length = (it.task2Obs2Length + delta).coerceIn(30, 100)) }
    }

    fun updateTargetInfo(obstacleId: Int, targetId: Int, imageUrl: String?) {
        _uiState.update { state ->
            val t2Obs1Target = if (obstacleId == 1) targetId else state.task2Obs1TargetId
            val t2Obs2Target = if (obstacleId == 2) targetId else state.task2Obs2TargetId

            val t2Obs1Arrow = if ((obstacleId == 1) && (targetId in listOf(38, 39))) {
                if (targetId == 38) ArrowDir.RIGHT else ArrowDir.LEFT
            } else state.task2Obs1Arrow
            val t2Obs2Arrow = if ((obstacleId == 2) && (targetId in listOf(38, 39))) {
                if (targetId == 38) ArrowDir.RIGHT else ArrowDir.LEFT
            } else state.task2Obs2Arrow

            val defaultImageUrl = "${state.rpiServerBaseUrl.trimEnd('/')}/images/$obstacleId?t=${System.currentTimeMillis()}"
            val fullImageUrl = imageUrl?.let { img ->
                if (img.startsWith("http://") || img.startsWith("https://")) img
                else "${state.rpiServerBaseUrl.trimEnd('/')}/${img.trimStart('/')}"
            } ?: defaultImageUrl

            val t2Obs1Image = if (obstacleId == 1) fullImageUrl else state.task2Obs1ImageUrl
            val t2Obs2Image = if (obstacleId == 2) fullImageUrl else state.task2Obs2ImageUrl

            val newRobotStep = when {
                (t2Obs1Target != null) && (t2Obs2Target != null) -> Task2RobotStep.END
                obstacleId == 2 -> Task2RobotStep.AT_OBS_2
                obstacleId == 1 -> Task2RobotStep.AT_OBS_1
                else -> state.robotStep
            }

            state.copy(
                task2Obs1TargetId = t2Obs1Target,
                task2Obs2TargetId = t2Obs2Target,
                task2Obs1Arrow = t2Obs1Arrow,
                task2Obs2Arrow = t2Obs2Arrow,
                task2Obs1ImageUrl = t2Obs1Image,
                task2Obs2ImageUrl = t2Obs2Image,
                robotStep = newRobotStep,
            )
        }
    }

    fun resetTask2Data() {
        _uiState.update {
            it.copy(
                task2Obs1TargetId = null,
                task2Obs2TargetId = null,
                task2Obs1Arrow = null,
                task2Obs2Arrow = null,
                task2Obs1ImageUrl = null,
                task2Obs2ImageUrl = null,
                robotStep = Task2RobotStep.START,
            )
        }
    }
}
