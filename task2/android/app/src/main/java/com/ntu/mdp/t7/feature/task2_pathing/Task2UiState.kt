package com.ntu.mdp.t7.feature.task2_pathing

import com.ntu.mdp.t7.core.model.ArrowDir

data class Task2UiState(
    val task2Distance1: Int = 60,
    val task2Distance2: Int = 60,
    val task2Obs2Length: Int = 30,
    val task2Obs1Arrow: ArrowDir? = null,
    val task2Obs2Arrow: ArrowDir? = null,
    val task2Obs1TargetId: Int? = null,
    val task2Obs2TargetId: Int? = null,
    val task2Obs1ImageUrl: String? = null,
    val task2Obs2ImageUrl: String? = null,
    val robotStep: Task2RobotStep = Task2RobotStep.START,
    val rpiServerBaseUrl: String = "http://192.168.1.1:5001/",
)

enum class Task2RobotStep {
    START,
    AT_OBS_1,
    AT_OBS_2,
    END
}
