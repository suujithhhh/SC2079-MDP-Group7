@file:OptIn(kotlinx.coroutines.ExperimentalCoroutinesApi::class)

package com.example.myapplication.arena

import com.example.myapplication.bluetooth.FakeTransport
import com.example.myapplication.protocol.ProtocolSerializer
import kotlinx.coroutines.Dispatchers
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

class ArenaViewModelTest {

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
    fun testStartsIdleNotConnectedState() = runTest(dispatcher) {
        val viewModel = ArenaViewModel(FakeTransport())
        dispatcher.scheduler.advanceUntilIdle()

        assertEquals(LinkState.Idle, viewModel.uiState.value.link)
    }

    @Test
    fun testIncomingRobotLineUpdatesRobotPose() = runTest(dispatcher) {
        val transport = FakeTransport()
        val viewModel = ArenaViewModel(transport)
        dispatcher.scheduler.advanceUntilIdle()

        transport.emitIncoming("ROBOT,3,5,E")
        dispatcher.scheduler.advanceUntilIdle()

        val state = viewModel.uiState.value
        assertEquals(3, state.robot.x)
        assertEquals(5, state.robot.y)
        assertEquals(Direction.EAST, state.robot.direction)
        assertTrue(state.statusLog.last().message.contains("03,05"))
    }

    @Test
    fun testIncomingTargetLineUpdatesMatchingObstacle() = runTest(dispatcher) {
        val transport = FakeTransport()
        val viewModel = ArenaViewModel(transport)
        dispatcher.scheduler.advanceUntilIdle()
        viewModel.onEditToggleClick()
        viewModel.onGridClick(GridCell(x = 5, y = 5))
        viewModel.onFaceSelected(Direction.NORTH)
        viewModel.onFaceDialogConfirm()
        dispatcher.scheduler.advanceUntilIdle()

        transport.emitIncoming("TARGET,B1,20,W")
        dispatcher.scheduler.advanceUntilIdle()

        val obstacle = viewModel.uiState.value.obstacles.first { it.id == 1 }
        assertEquals(20, obstacle.targetId)
        assertEquals(Direction.WEST, obstacle.face)
    }

    @Test
    fun testIncomingMsgLineAppendsStatusEntry() = runTest(dispatcher) {
        val transport = FakeTransport()
        val viewModel = ArenaViewModel(transport)
        dispatcher.scheduler.advanceUntilIdle()

        transport.emitIncoming("MSG,[Moving]")
        dispatcher.scheduler.advanceUntilIdle()

        assertEquals("Moving", viewModel.uiState.value.statusLog.last().message)
    }

    @Test
    fun testMalformedIncomingLineDoesNotCrash() = runTest(dispatcher) {
        val transport = FakeTransport()
        val viewModel = ArenaViewModel(transport)
        dispatcher.scheduler.advanceUntilIdle()
        val before = viewModel.uiState.value

        transport.emitIncoming("NOT,A,REAL,COMMAND,AT,ALL")
        dispatcher.scheduler.advanceUntilIdle()

        val after = viewModel.uiState.value
        assertEquals(before.robot, after.robot)
        assertEquals(before.obstacles, after.obstacles)
    }

    @Test
    fun testMalformedIncomingLineSurfacedInStatusLog() = runTest(dispatcher) {
        val transport = FakeTransport()
        val viewModel = ArenaViewModel(transport)
        dispatcher.scheduler.advanceUntilIdle()

        transport.emitIncoming("NOT,A,REAL,COMMAND,AT,ALL")
        dispatcher.scheduler.advanceUntilIdle()

        assertEquals("[Received] NOT,A,REAL,COMMAND,AT,ALL", viewModel.uiState.value.statusLog.last().message)
    }

    @Test
    fun testTargetWithInvalidImageIdShowsStatus() = runTest(dispatcher) {
        val transport = FakeTransport()
        val viewModel = ArenaViewModel(transport)
        dispatcher.scheduler.advanceUntilIdle()
        viewModel.onEditToggleClick()
        viewModel.onGridClick(GridCell(x = 5, y = 5))
        viewModel.onFaceSelected(Direction.NORTH)
        viewModel.onFaceDialogConfirm()
        dispatcher.scheduler.advanceUntilIdle()

        transport.emitIncoming("TARGET,B1,1")
        dispatcher.scheduler.advanceUntilIdle()

        val state = viewModel.uiState.value
        assertEquals("Target ID 1 is invalid", state.statusLog.last().message)
        assertEquals(StatusLevel.ALERT, state.statusLog.last().level)
        assertEquals(null, state.obstacles.first { it.id == 1 }.targetId)
    }

    @Test
    fun testSubmittingObstaclesSendsPlanRequest() = runTest(dispatcher) {
        val transport = FakeTransport()
        val viewModel = ArenaViewModel(transport)
        dispatcher.scheduler.advanceUntilIdle()
        transport.connectTo("98:D3:31:FB:2A:C4", "HC-05")
        dispatcher.scheduler.advanceUntilIdle()
        viewModel.onEditToggleClick() // LOCKED -> EDITING
        viewModel.onGridClick(GridCell(x = 5, y = 5))
        viewModel.onFaceSelected(Direction.NORTH)
        viewModel.onFaceDialogConfirm()
        dispatcher.scheduler.advanceUntilIdle()

        viewModel.onEditToggleClick() // EDITING -> LOCKED, generates + sends
        dispatcher.scheduler.advanceUntilIdle()

        val expected = ProtocolSerializer.planRequest(viewModel.uiState.value.robot, viewModel.uiState.value.obstacles)
        assertEquals(listOf(expected), transport.sentLines)
        assertEquals(listOf("→ $expected"), viewModel.uiState.value.rawProtocolLog)
        assertEquals(EditMode.LOCKED, viewModel.uiState.value.editMode)
    }

    @Test
    fun testIncomingLinesAppendedToRawProtocolLog() = runTest(dispatcher) {
        val transport = FakeTransport()
        val viewModel = ArenaViewModel(transport)
        dispatcher.scheduler.advanceUntilIdle()

        transport.emitIncoming("ROBOT,3,5,E")
        transport.emitIncoming("MSG,[Moving]")
        dispatcher.scheduler.advanceUntilIdle()

        assertEquals(
            listOf("← ROBOT,3,5,E", "← MSG,[Moving]"),
            viewModel.uiState.value.rawProtocolLog,
        )
    }

    @Test
    fun testRawProtocolLogAccumulatesSentAndReceivedLines() = runTest(dispatcher) {
        val transport = FakeTransport()
        val viewModel = ArenaViewModel(transport)
        dispatcher.scheduler.advanceUntilIdle()
        transport.connectTo("98:D3:31:FB:2A:C4", "HC-05")
        dispatcher.scheduler.advanceUntilIdle()

        transport.emitIncoming("ROBOT,3,5,E")
        dispatcher.scheduler.advanceUntilIdle()
        viewModel.onEditToggleClick() // LOCKED -> EDITING
        viewModel.onEditToggleClick() // EDITING -> LOCKED, generates + sends
        dispatcher.scheduler.advanceUntilIdle()
        transport.emitIncoming("MSG,[Done]")
        dispatcher.scheduler.advanceUntilIdle()

        val sentPlanRequest = transport.sentLines.single()
        assertEquals(
            listOf("← ROBOT,3,5,E", "→ $sentPlanRequest", "← MSG,[Done]"),
            viewModel.uiState.value.rawProtocolLog,
        )
    }

    @Test
    fun testOnScanClickOpensDevicePicker() = runTest(dispatcher) {
        val viewModel = ArenaViewModel(FakeTransport())
        dispatcher.scheduler.advanceUntilIdle()

        viewModel.onScanClick()

        assertEquals(LinkState.Scanning, viewModel.uiState.value.link)
        assertTrue(viewModel.devicePicker.value.isVisible)
        assertTrue(viewModel.devicePicker.value.isScanning)
    }

    @Test
    fun testConnectingCarriesSignalQuality() = runTest(dispatcher) {
        val transport = FakeTransport()
        val viewModel = ArenaViewModel(transport)
        dispatcher.scheduler.advanceUntilIdle()

        viewModel.onDeviceFound(name = "HC-05", address = "98:D3:31:FB:2A:C4", rssi = -42)
        viewModel.onDeviceSelected(DiscoveredDevice(name = "HC-05", address = "98:D3:31:FB:2A:C4", signalStrength = -42))
        viewModel.onConnectClick()
        dispatcher.scheduler.advanceUntilIdle()

        val link = viewModel.uiState.value.link as LinkState.Connected
        assertEquals(SignalQuality.STRONG, link.signal)
    }

    @Test
    fun testOnDeviceSelectedConnectsDirectly() = runTest(dispatcher) {
        val transport = FakeTransport()
        val viewModel = ArenaViewModel(transport)
        dispatcher.scheduler.advanceUntilIdle()

        viewModel.onDeviceSelected(
            DiscoveredDevice(name = "AMD-Tool-07", address = "34:14:B5:8A:11:9C", signalStrength = -61, isPaired = false),
        )
        dispatcher.scheduler.advanceUntilIdle()

        // With the fast-connect optimization, FakeTransport immediately accepts the connection
        assertTrue(viewModel.uiState.value.link is LinkState.Connected)
        val link = viewModel.uiState.value.link as LinkState.Connected
        assertEquals("AMD-Tool-07", link.deviceName)
        assertEquals("34:14:B5:8A:11:9C", link.address)
    }

    @Test
    fun testOnDisconnectClickDisconnectsTransport() = runTest(dispatcher) {
        val transport = FakeTransport()
        val viewModel = ArenaViewModel(transport)
        dispatcher.scheduler.advanceUntilIdle()
        transport.connectTo("98:D3:31:FB:2A:C4", "HC-05")
        dispatcher.scheduler.advanceUntilIdle()

        viewModel.onDisconnectClick()
        dispatcher.scheduler.advanceUntilIdle()

        assertEquals(LinkState.Idle, viewModel.uiState.value.link)
    }

    @Test
    fun testGivesUpAndAlertsAfterFailedReconnects() = runTest(dispatcher) {
        val transport = FakeTransport()
        val viewModel = ArenaViewModel(transport)
        dispatcher.scheduler.advanceUntilIdle()
        transport.connectTo("98:D3:31:FB:2A:C4", "HC-05")
        dispatcher.scheduler.advanceUntilIdle()
        assertTrue(viewModel.uiState.value.link is LinkState.Connected)
        val attemptsBeforeFailure = transport.connectAttempts

        transport.failConnections = true
        transport.disconnect()
        dispatcher.scheduler.runCurrent()
        assertTrue(viewModel.uiState.value.link is LinkState.Lost)

        // No attempt should fire before the first 1.5s interval elapses.
        dispatcher.scheduler.advanceTimeBy(1_499L)
        dispatcher.scheduler.runCurrent()
        assertEquals(0, transport.connectAttempts - attemptsBeforeFailure)

        // Each subsequent 1.5s tick should fire exactly one more attempt.
        repeat(10) { index ->
            dispatcher.scheduler.advanceTimeBy(1L)
            dispatcher.scheduler.runCurrent()
            assertEquals(index + 1, transport.connectAttempts - attemptsBeforeFailure)
            dispatcher.scheduler.advanceTimeBy(1_499L)
            dispatcher.scheduler.runCurrent()
        }

        assertEquals(10, transport.connectAttempts - attemptsBeforeFailure)
        assertTrue(viewModel.uiState.value.link is LinkState.Lost)
        assertTrue(
            viewModel.uiState.value.statusLog.any {
                it.message.contains("Could not reconnect after 10 attempts")
            },
        )
    }

    @Test
    fun testReconnectLoopStopsAfterManualConnect() = runTest(dispatcher) {
        val transport = FakeTransport()
        val viewModel = ArenaViewModel(transport)
        dispatcher.scheduler.advanceUntilIdle()
        transport.connectTo("98:D3:31:FB:2A:C4", "HC-05")
        dispatcher.scheduler.advanceUntilIdle()

        transport.failConnections = true
        transport.disconnect()
        dispatcher.scheduler.advanceUntilIdle()
        assertTrue(viewModel.uiState.value.link is LinkState.Lost)

        transport.failConnections = false
        viewModel.onConnectClick()
        dispatcher.scheduler.advanceUntilIdle()
        assertTrue(viewModel.uiState.value.link is LinkState.Connected)
        val attemptsAtSuccess = transport.connectAttempts

        dispatcher.scheduler.advanceTimeBy(20_000L)
        dispatcher.scheduler.advanceUntilIdle()

        assertEquals(attemptsAtSuccess, transport.connectAttempts)
        assertTrue(viewModel.uiState.value.link is LinkState.Connected)
    }

    @Test
    fun testOnObstacleClickTogglesSelection() = runTest(dispatcher) {
        val viewModel = ArenaViewModel(FakeTransport())
        dispatcher.scheduler.advanceUntilIdle()
        viewModel.onEditToggleClick()
        viewModel.onGridClick(GridCell(x = 5, y = 5))
        viewModel.onFaceSelected(Direction.NORTH)
        viewModel.onFaceDialogConfirm()
        viewModel.onGridClick(GridCell(x = 10, y = 10))
        viewModel.onFaceSelected(Direction.NORTH)
        viewModel.onFaceDialogConfirm()
        viewModel.onEditToggleClick() // switch back to LOCKED to allow selection
        dispatcher.scheduler.advanceUntilIdle()
        val target = viewModel.uiState.value.obstacles.first { it.id == 1 }

        viewModel.onObstacleClick(target)
        dispatcher.scheduler.advanceUntilIdle()

        val selectedAfterClick = viewModel.uiState.value.obstacles
        assertEquals(true, selectedAfterClick.first { it.id == 1 }.isSelected)
        assertEquals(false, selectedAfterClick.first { it.id == 2 }.isSelected)

        viewModel.onObstacleClick(target)
        dispatcher.scheduler.advanceUntilIdle()

        assertTrue(viewModel.uiState.value.obstacles.none { it.isSelected })
    }

    @Test
    fun testOnRobotDragEndRelocatesFootprint() = runTest(dispatcher) {
        val viewModel = ArenaViewModel(FakeTransport())
        dispatcher.scheduler.advanceUntilIdle()

        viewModel.onRobotDragEnd(GridCell(x = 5, y = 5))
        dispatcher.scheduler.advanceUntilIdle()

        assertEquals(RobotPose(x = 5, y = 5, direction = Direction.NORTH), viewModel.uiState.value.robot)
    }

    @Test
    fun testOnRobotDragEndNoOpWhenOverlapping() = runTest(dispatcher) {
        val viewModel = ArenaViewModel(FakeTransport())
        dispatcher.scheduler.advanceUntilIdle()
        viewModel.onEditToggleClick()
        viewModel.onGridClick(GridCell(x = 5, y = 5))
        viewModel.onFaceSelected(Direction.NORTH)
        viewModel.onFaceDialogConfirm()
        dispatcher.scheduler.advanceUntilIdle()
        val before = viewModel.uiState.value.robot

        viewModel.onRobotDragEnd(GridCell(x = 5, y = 5))
        dispatcher.scheduler.advanceUntilIdle()

        assertEquals(before, viewModel.uiState.value.robot)
    }

    @Test
    fun testOnGridClickCreatesObstacleAndOpensDialog() = runTest(dispatcher) {
        val viewModel = ArenaViewModel(FakeTransport())
        dispatcher.scheduler.advanceUntilIdle()
        viewModel.onEditToggleClick()

        viewModel.onGridClick(GridCell(5, 5))
        dispatcher.scheduler.advanceUntilIdle()

        val state = viewModel.uiState.value
        assertEquals(1, state.obstacles.size)
        assertEquals(1, state.obstacles.first().id)
        assertEquals(1, state.obstacleFaceDialogTarget)
        assertTrue(state.isNewPlacement)
    }

    @Test
    fun testOnGridClickOccupiedCellIsNoOp() = runTest(dispatcher) {
        val viewModel = ArenaViewModel(FakeTransport())
        dispatcher.scheduler.advanceUntilIdle()
        viewModel.onEditToggleClick()
        
        viewModel.onGridClick(GridCell(5, 5))
        viewModel.onFaceDialogConfirm()
        
        viewModel.onGridClick(GridCell(5, 5)) // occupied by obstacle 1
        dispatcher.scheduler.advanceUntilIdle()

        val state = viewModel.uiState.value
        assertEquals(1, state.obstacles.size)
        assertEquals(null, state.obstacleFaceDialogTarget)
    }

    @Test
    fun testOnGridClickOutsideEditingModeIsNoOp() = runTest(dispatcher) {
        val viewModel = ArenaViewModel(FakeTransport())
        dispatcher.scheduler.advanceUntilIdle()

        viewModel.onGridClick(GridCell(5, 5))
        dispatcher.scheduler.advanceUntilIdle()

        val state = viewModel.uiState.value
        assertEquals(0, state.obstacles.size)
        assertEquals(null, state.obstacleFaceDialogTarget)
    }

    @Test
    fun testOnFaceSelectedUpdatesFace() = runTest(dispatcher) {
        val viewModel = ArenaViewModel(FakeTransport())
        dispatcher.scheduler.advanceUntilIdle()
        viewModel.onEditToggleClick()
        
        viewModel.onGridClick(GridCell(5, 5))
        viewModel.onFaceSelected(Direction.NORTH)
        dispatcher.scheduler.advanceUntilIdle()

        val state = viewModel.uiState.value
        assertEquals(Direction.NORTH, state.obstacles.first().face)
        // Dialog should remain open
        assertEquals(1, state.obstacleFaceDialogTarget)
    }

    @Test
    fun testOnFaceDialogConfirmClosesDialog() = runTest(dispatcher) {
        val viewModel = ArenaViewModel(FakeTransport())
        dispatcher.scheduler.advanceUntilIdle()
        viewModel.onEditToggleClick()
        
        viewModel.onGridClick(GridCell(5, 5))
        viewModel.onFaceDialogConfirm()
        dispatcher.scheduler.advanceUntilIdle()

        val state = viewModel.uiState.value
        assertEquals(null, state.obstacleFaceDialogTarget)
        assertFalse(state.isNewPlacement)
    }

    @Test
    fun testOnFaceDialogDismissRollsBackNewPlacement() = runTest(dispatcher) {
        val viewModel = ArenaViewModel(FakeTransport())
        dispatcher.scheduler.advanceUntilIdle()
        viewModel.onEditToggleClick()
        
        // Place first obstacle
        viewModel.onGridClick(GridCell(5, 5))
        viewModel.onFaceDialogConfirm()

        // Place second obstacle but dismiss
        viewModel.onGridClick(GridCell(7, 7))
        viewModel.onFaceDialogDismiss()
        dispatcher.scheduler.advanceUntilIdle()

        val state = viewModel.uiState.value
        assertEquals(1, state.obstacles.size)
        assertEquals(1, state.obstacles.first().id)
        assertEquals(null, state.obstacleFaceDialogTarget)
    }

    @Test
    fun testOnFaceDialogDismissDoesNotRollBackExistingEdit() = runTest(dispatcher) {
        val viewModel = ArenaViewModel(FakeTransport())
        dispatcher.scheduler.advanceUntilIdle()
        viewModel.onEditToggleClick()
        
        viewModel.onGridClick(GridCell(5, 5))
        viewModel.onFaceDialogConfirm()

        // Click again to edit
        viewModel.onObstacleClick(viewModel.uiState.value.obstacles.first())
        viewModel.onFaceDialogDismiss()
        dispatcher.scheduler.advanceUntilIdle()

        val state = viewModel.uiState.value
        assertEquals(1, state.obstacles.size)
    }

    @Test
    fun testOnDeleteObstacleClickRemovesAndRenumbers() = runTest(dispatcher) {
        val viewModel = ArenaViewModel(FakeTransport())
        dispatcher.scheduler.advanceUntilIdle()
        viewModel.onEditToggleClick()
        
        viewModel.onGridClick(GridCell(5, 5))
        viewModel.onFaceDialogConfirm()
        viewModel.onGridClick(GridCell(7, 7))
        viewModel.onFaceDialogConfirm()
        viewModel.onGridClick(GridCell(9, 9))
        viewModel.onFaceDialogConfirm()

        viewModel.onDeleteObstacleClick(2)
        dispatcher.scheduler.advanceUntilIdle()

        val state = viewModel.uiState.value
        assertEquals(2, state.obstacles.size)
        assertEquals(1, state.obstacles[0].id)
        assertEquals(2, state.obstacles[1].id)
    }

    @Test
    fun testSuccessfulRobotDragEndSetsPickerVisible() = runTest(dispatcher) {
        val viewModel = ArenaViewModel(FakeTransport())
        dispatcher.scheduler.advanceUntilIdle()

        viewModel.onRobotDragEnd(GridCell(x = 5, y = 5))
        dispatcher.scheduler.advanceUntilIdle()

        assertEquals(RobotPose(x = 5, y = 5, direction = Direction.NORTH), viewModel.uiState.value.robot)
        assertTrue(viewModel.uiState.value.robotDirectionPickerVisible)
    }

    @Test
    fun testRobotDragEndRefusedDoesNotSetPickerVisible() = runTest(dispatcher) {
        val viewModel = ArenaViewModel(FakeTransport())
        dispatcher.scheduler.advanceUntilIdle()
        viewModel.onEditToggleClick()
        viewModel.onGridClick(GridCell(x = 5, y = 5))
        viewModel.onFaceSelected(Direction.NORTH)
        viewModel.onFaceDialogConfirm()
        dispatcher.scheduler.advanceUntilIdle()

        viewModel.onRobotDragEnd(GridCell(x = 5, y = 5)) // overlaps the obstacle just placed
        dispatcher.scheduler.advanceUntilIdle()

        assertEquals(false, viewModel.uiState.value.robotDirectionPickerVisible)
    }

    @Test
    fun testOnRobotFaceSelectedUpdatesDirection() = runTest(dispatcher) {
        val viewModel = ArenaViewModel(FakeTransport())
        dispatcher.scheduler.advanceUntilIdle()
        viewModel.onRobotDragEnd(GridCell(x = 5, y = 5))

        viewModel.onRobotFaceSelected(Direction.WEST)
        dispatcher.scheduler.advanceUntilIdle()

        assertEquals(RobotPose(x = 5, y = 5, direction = Direction.WEST), viewModel.uiState.value.robot)
        assertEquals(false, viewModel.uiState.value.robotDirectionPickerVisible)
    }

    @Test
    fun testRedraggingRobotReanchors() = runTest(dispatcher) {
        val viewModel = ArenaViewModel(FakeTransport())
        dispatcher.scheduler.advanceUntilIdle()
        viewModel.onRobotDragEnd(GridCell(x = 5, y = 5))

        viewModel.onRobotDragEnd(GridCell(x = 8, y = 8))
        dispatcher.scheduler.advanceUntilIdle()

        assertEquals(RobotPose(x = 8, y = 8, direction = Direction.NORTH), viewModel.uiState.value.robot)
        assertTrue(viewModel.uiState.value.robotDirectionPickerVisible)
    }

    @Test
    fun testOnClearGridClickClearsDirectionPicker() = runTest(dispatcher) {
        val viewModel = ArenaViewModel(FakeTransport())
        dispatcher.scheduler.advanceUntilIdle()
        viewModel.onRobotDragEnd(GridCell(x = 5, y = 5))

        viewModel.onClearGridClick()
        dispatcher.scheduler.advanceUntilIdle()

        assertEquals(false, viewModel.uiState.value.robotDirectionPickerVisible)
    }

    @Test
    fun testIncomingRobotLineDoesNotSetDirectionPickerVisible() = runTest(dispatcher) {
        val transport = FakeTransport()
        val viewModel = ArenaViewModel(transport)
        dispatcher.scheduler.advanceUntilIdle()

        transport.emitIncoming("ROBOT,3,5,E")
        dispatcher.scheduler.advanceUntilIdle()

        assertEquals(false, viewModel.uiState.value.robotDirectionPickerVisible)
    }

    @Test
    fun testOnRobotTappedOpensDirectionPicker() = runTest(dispatcher) {
        val viewModel = ArenaViewModel(FakeTransport())
        dispatcher.scheduler.advanceUntilIdle()

        viewModel.onRobotTapped()
        dispatcher.scheduler.advanceUntilIdle()

        assertTrue(viewModel.uiState.value.robotDirectionPickerVisible)
    }

    @Test
    fun testOnRobotTappedClosesDirectionPicker() = runTest(dispatcher) {
        val viewModel = ArenaViewModel(FakeTransport())
        dispatcher.scheduler.advanceUntilIdle()
        viewModel.onRobotTapped()

        viewModel.onRobotTapped()
        dispatcher.scheduler.advanceUntilIdle()

        assertEquals(false, viewModel.uiState.value.robotDirectionPickerVisible)
    }

    @Test
    fun testOnRobotFaceSelectedClosesPickerWhenOpenedViaTap() = runTest(dispatcher) {
        val viewModel = ArenaViewModel(FakeTransport())
        dispatcher.scheduler.advanceUntilIdle()
        viewModel.onRobotTapped()

        viewModel.onRobotFaceSelected(Direction.EAST)
        dispatcher.scheduler.advanceUntilIdle()

        assertEquals(Direction.EAST, viewModel.uiState.value.robot.direction)
        assertEquals(false, viewModel.uiState.value.robotDirectionPickerVisible)
    }

    @Test
    fun testSubmitSucceedsWhileDirectionPickerIsOpen() = runTest(dispatcher) {
        val viewModel = ArenaViewModel(FakeTransport())
        dispatcher.scheduler.advanceUntilIdle()
        viewModel.onEditToggleClick() // LOCKED -> EDITING
        viewModel.onRobotTapped() // open the picker without dragging
        dispatcher.scheduler.advanceUntilIdle()

        viewModel.onEditToggleClick() // EDITING -> LOCKED
        dispatcher.scheduler.advanceUntilIdle()

        assertEquals(EditMode.LOCKED, viewModel.uiState.value.editMode)
        assertEquals(false, viewModel.uiState.value.robotDirectionPickerVisible)
    }

    // --- Type obstacles (text entry) ---

    /** Presses keypad keys one by one. A face letter starts the next line by itself, so there
     * is no new-line key to press; stray characters are ignored by the ViewModel. */
    private fun ArenaViewModel.typeText(text: String) {
        text.forEach { onTypeObstaclesChar(it) }
    }

    @Test
    fun testStartEditingOpensInEditMode() = runTest(dispatcher) {
        val viewModel = ArenaViewModel(FakeTransport(), startEditing = true)
        dispatcher.scheduler.advanceUntilIdle()

        assertEquals(EditMode.EDITING, viewModel.uiState.value.editMode)
        assertEquals(EditMode.LOCKED, ArenaViewModel(FakeTransport()).uiState.value.editMode)
    }

    @Test
    fun testTypeObstaclesClickIsIgnoredWhileLocked() = runTest(dispatcher) {
        val viewModel = ArenaViewModel(FakeTransport())
        dispatcher.scheduler.advanceUntilIdle()

        viewModel.onTypeObstaclesClick()

        assertFalse(viewModel.uiState.value.typeObstaclesVisible)
    }

    @Test
    fun testTypeObstaclesClickPrefillsDraftFromGrid() = runTest(dispatcher) {
        val viewModel = ArenaViewModel(FakeTransport(), startEditing = true)
        dispatcher.scheduler.advanceUntilIdle()
        viewModel.onObstaclePlacedWithFace(GridCell(5, 13), Direction.WEST)
        viewModel.onObstaclePlacedWithFace(GridCell(5, 7), Direction.SOUTH)

        viewModel.onTypeObstaclesClick()

        assertTrue(viewModel.uiState.value.typeObstaclesVisible)
        assertEquals("5,13l\n5,7d\n", viewModel.uiState.value.typeObstaclesDraft)
    }

    @Test
    fun testTypeObstaclesCharsBuildTheDraftAndIgnoreOtherKeys() = runTest(dispatcher) {
        val viewModel = ArenaViewModel(FakeTransport(), startEditing = true)
        dispatcher.scheduler.advanceUntilIdle()
        viewModel.onTypeObstaclesClick()

        viewModel.typeText("5,13")
        viewModel.onTypeObstaclesChar('x') // not a keypad key
        viewModel.onTypeObstaclesChar(' ')
        viewModel.onTypeObstaclesChar('\n')

        assertEquals("5,13", viewModel.uiState.value.typeObstaclesDraft)
    }

    @Test
    fun testFaceLetterEndsTheLineAndStartsTheNextOne() = runTest(dispatcher) {
        val viewModel = ArenaViewModel(FakeTransport(), startEditing = true)
        dispatcher.scheduler.advanceUntilIdle()
        viewModel.onTypeObstaclesClick()

        viewModel.typeText("3,4L5,2r")

        // Capital letters are lower-cased, and each face letter is followed by a line break.
        assertEquals("3,4l\n5,2r\n", viewModel.uiState.value.typeObstaclesDraft)
    }

    @Test
    fun testTheOldCommaBeforeTheFaceLetterStillWorks() = runTest(dispatcher) {
        val viewModel = ArenaViewModel(FakeTransport(), startEditing = true)
        dispatcher.scheduler.advanceUntilIdle()
        viewModel.onTypeObstaclesClick()
        viewModel.typeText("3,4,l")
        assertEquals("3,4,l\n", viewModel.uiState.value.typeObstaclesDraft)

        viewModel.onTypeObstaclesPlace()

        assertEquals(listOf(Obstacle(id = 1, x = 3, y = 4, face = Direction.WEST)), viewModel.uiState.value.obstacles)
    }

    @Test
    fun testBackspaceAfterAFaceLetterRemovesTheLetterAndTheLineBreakTogether() = runTest(dispatcher) {
        val viewModel = ArenaViewModel(FakeTransport(), startEditing = true)
        dispatcher.scheduler.advanceUntilIdle()
        viewModel.onTypeObstaclesClick()
        viewModel.typeText("3,4l")
        assertEquals("3,4l\n", viewModel.uiState.value.typeObstaclesDraft)

        viewModel.onTypeObstaclesBackspace() // one tap undoes the face letter and its line break
        assertEquals("3,4", viewModel.uiState.value.typeObstaclesDraft)

        viewModel.onTypeObstaclesBackspace()
        assertEquals("3,", viewModel.uiState.value.typeObstaclesDraft)

        viewModel.onTypeObstaclesBackspace()
        viewModel.onTypeObstaclesBackspace()
        viewModel.onTypeObstaclesBackspace() // nothing left to delete
        assertEquals("", viewModel.uiState.value.typeObstaclesDraft)
    }

    @Test
    fun testTypeObstaclesClearEmptiesTheDraft() = runTest(dispatcher) {
        val viewModel = ArenaViewModel(FakeTransport(), startEditing = true)
        dispatcher.scheduler.advanceUntilIdle()
        viewModel.onTypeObstaclesClick()
        viewModel.typeText("3,4l5,2")

        viewModel.onTypeObstaclesClear()

        assertEquals("", viewModel.uiState.value.typeObstaclesDraft)
    }

    @Test
    fun testTypeObstaclesPlaceReplacesGridWithIdsByLineOrder() = runTest(dispatcher) {
        val viewModel = ArenaViewModel(FakeTransport(), startEditing = true)
        dispatcher.scheduler.advanceUntilIdle()
        viewModel.onObstaclePlacedWithFace(GridCell(9, 9), Direction.NORTH) // will be replaced
        viewModel.onTypeObstaclesClick()
        viewModel.onTypeObstaclesClear()
        viewModel.typeText("5,13l5,7d12,9r15,15d15,4u")

        viewModel.onTypeObstaclesPlace()

        val state = viewModel.uiState.value
        assertFalse(state.typeObstaclesVisible)
        assertEquals("", state.typeObstaclesDraft)
        assertEquals(
            listOf(
                Obstacle(id = 1, x = 5, y = 13, face = Direction.WEST),
                Obstacle(id = 2, x = 5, y = 7, face = Direction.SOUTH),
                Obstacle(id = 3, x = 12, y = 9, face = Direction.EAST),
                Obstacle(id = 4, x = 15, y = 15, face = Direction.SOUTH),
                Obstacle(id = 5, x = 15, y = 4, face = Direction.NORTH),
            ),
            state.obstacles,
        )
        assertTrue(state.statusLog.any { it.message.contains("Placed 5 obstacle(s) from text (replaced 1)") })
    }

    @Test
    fun testTypeObstaclesPlaceDoesNothingWhenAnyLineIsInvalid() = runTest(dispatcher) {
        val viewModel = ArenaViewModel(FakeTransport(), startEditing = true)
        dispatcher.scheduler.advanceUntilIdle()
        viewModel.onObstaclePlacedWithFace(GridCell(9, 9), Direction.NORTH)
        viewModel.onTypeObstaclesClick()
        viewModel.onTypeObstaclesClear()
        viewModel.typeText("5,13l20,7d") // x=20 is outside the grid

        viewModel.onTypeObstaclesPlace()

        val state = viewModel.uiState.value
        assertTrue(state.typeObstaclesVisible)
        assertEquals(listOf(Obstacle(id = 1, x = 9, y = 9, face = Direction.NORTH)), state.obstacles)
    }

    @Test
    fun testTypeObstaclesPlaceRejectsAnObstacleOnTheRobot() = runTest(dispatcher) {
        val viewModel = ArenaViewModel(FakeTransport(), startEditing = true)
        dispatcher.scheduler.advanceUntilIdle()
        viewModel.onTypeObstaclesClick()
        viewModel.typeText("2,2u") // default robot covers x1-2, y1-3

        viewModel.onTypeObstaclesPlace()

        assertTrue(viewModel.uiState.value.typeObstaclesVisible)
        assertTrue(viewModel.uiState.value.obstacles.isEmpty())
    }

    @Test
    fun testTypeObstaclesPlaceRejectsAHalfTypedLine() = runTest(dispatcher) {
        val viewModel = ArenaViewModel(FakeTransport(), startEditing = true)
        dispatcher.scheduler.advanceUntilIdle()
        viewModel.onTypeObstaclesClick()
        viewModel.typeText("5,13l5,7") // second obstacle has no face letter yet

        viewModel.onTypeObstaclesPlace()

        assertTrue(viewModel.uiState.value.typeObstaclesVisible)
        assertTrue(viewModel.uiState.value.obstacles.isEmpty())
    }

    @Test
    fun testTypeObstaclesDismissDiscardsTheDraftAndKeepsTheGrid() = runTest(dispatcher) {
        val viewModel = ArenaViewModel(FakeTransport(), startEditing = true)
        dispatcher.scheduler.advanceUntilIdle()
        viewModel.onObstaclePlacedWithFace(GridCell(9, 9), Direction.NORTH)
        viewModel.onTypeObstaclesClick()
        viewModel.typeText("1,1u")

        viewModel.onTypeObstaclesDismiss()

        assertFalse(viewModel.uiState.value.typeObstaclesVisible)
        assertEquals("", viewModel.uiState.value.typeObstaclesDraft)
        assertEquals(1, viewModel.uiState.value.obstacles.size)
    }

    @Test
    fun testSubmittingTypedObstaclesSendsTheSamePlanRequestAsTappedOnes() = runTest(dispatcher) {
        val transport = FakeTransport()
        val viewModel = ArenaViewModel(transport, startEditing = true)
        dispatcher.scheduler.advanceUntilIdle()
        transport.connectTo("98:D3:31:FB:2A:C4", "HC-05")
        dispatcher.scheduler.advanceUntilIdle()
        viewModel.onTypeObstaclesClick()
        viewModel.typeText("3,4l5,2r6,8u")
        viewModel.onTypeObstaclesPlace()

        viewModel.onEditToggleClick() // Submit
        dispatcher.scheduler.advanceUntilIdle()

        val robot = viewModel.uiState.value.robot
        val expected = ProtocolSerializer.planRequest(
            robot,
            listOf(
                Obstacle(id = 1, x = 3, y = 4, face = Direction.WEST),
                Obstacle(id = 2, x = 5, y = 2, face = Direction.EAST),
                Obstacle(id = 3, x = 6, y = 8, face = Direction.NORTH),
            ),
        )
        assertEquals(listOf(expected), transport.sentLines)
        assertTrue(expected.contains("\"face\":\"W\"") && expected.contains("\"face\":\"E\"") && expected.contains("\"face\":\"N\""))
        assertEquals(EditMode.LOCKED, viewModel.uiState.value.editMode)
    }
}
