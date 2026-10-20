@file:OptIn(ExperimentalCoroutinesApi::class)

package com.ntu.mdp.t7.feature.task1_arena

import com.ntu.mdp.t7.core.model.Direction
import com.ntu.mdp.t7.core.model.EditMode
import com.ntu.mdp.t7.core.model.Obstacle
import com.ntu.mdp.t7.core.model.RobotPose
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.ExperimentalCoroutinesApi
import kotlinx.coroutines.test.StandardTestDispatcher
import kotlinx.coroutines.test.resetMain
import kotlinx.coroutines.test.runTest
import kotlinx.coroutines.test.setMain
import org.junit.After
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertTrue
import org.junit.Before
import org.junit.Test

class Task1ViewModelTest {

    private val dispatcher = StandardTestDispatcher()

    @Before
    fun setUp() {
        Dispatchers.setMain(dispatcher)
    }

    @After
    fun tearDown() {
        Dispatchers.resetMain()
    }

    @Test
    fun testDefaultRobotPose() = runTest(dispatcher) {
        val viewModel = Task1ViewModel()
        val state = viewModel.uiState.value
        assertEquals(RobotPose(x = 2, y = 1, direction = Direction.NORTH), state.robot)
    }

    @Test
    fun testGridClickAddsObstacleInEditMode() = runTest(dispatcher) {
        val viewModel = Task1ViewModel()
        viewModel.onEditToggleClick { }
        viewModel.onGridClick(5, 5)

        val state = viewModel.uiState.value
        assertEquals(1, state.obstacles.size)
        assertEquals(5, state.obstacles[0].x)
        assertEquals(5, state.obstacles[0].y)
    }

    @Test
    fun testEditToggleRefusesWhenObstaclesMissingFaces() = runTest(dispatcher) {
        val viewModel = Task1ViewModel()
        viewModel.onEditToggleClick { }
        viewModel.onGridClick(5, 5)

        val success = viewModel.onEditToggleClick { }
        assertFalse(success)
        assertEquals(EditMode.EDITING, viewModel.uiState.value.editMode)
    }

    @Test
    fun testEditToggleSucceedsWhenAllObstaclesHaveFaces() = runTest(dispatcher) {
        val viewModel = Task1ViewModel()
        viewModel.onEditToggleClick { }
        viewModel.onGridClick(5, 5)
        viewModel.onFaceSelected(Direction.NORTH)
        viewModel.onFaceDialogConfirm()

        var wireMessageSent: String? = null
        val success = viewModel.onEditToggleClick { msg -> wireMessageSent = msg }

        assertTrue(success)
        assertEquals(EditMode.LOCKED, viewModel.uiState.value.editMode)
        assertTrue(viewModel.uiState.value.isPlanSent)
        assertTrue(wireMessageSent?.contains("PLAN_REQUEST") == true)
    }

    @Test
    fun testDeleteObstacleRenumbersRemainingObstacles() = runTest(dispatcher) {
        val viewModel = Task1ViewModel()
        viewModel.onEditToggleClick { }
        viewModel.onGridClick(5, 5)
        viewModel.onFaceSelected(Direction.NORTH)
        viewModel.onFaceDialogConfirm()

        viewModel.onGridClick(8, 8)
        viewModel.onFaceSelected(Direction.SOUTH)
        viewModel.onFaceDialogConfirm()

        assertEquals(2, viewModel.uiState.value.obstacles.size)

        viewModel.onDeleteObstacleClick(1)

        val remaining = viewModel.uiState.value.obstacles
        assertEquals(1, remaining.size)
        assertEquals(1, remaining[0].id)
        assertEquals(8, remaining[0].x)
    }
}
