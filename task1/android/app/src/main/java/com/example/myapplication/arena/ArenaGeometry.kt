package com.example.myapplication.arena

import kotlin.math.abs
import kotlin.math.floor
import kotlin.math.round

/** Top-left pixel/dp offset of a cell, in the coordinate space the UI draws in. */
data class GridOffset(val left: Float, val top: Float)

/**
 * Converts arena grid coordinates (origin bottom-left, y increasing upward, per
 * docs/design/protocol.md) to top-left drawing coordinates (origin top-left, y
 * increasing downward). Pulled out as a standalone, dependency-free function per
 * the protocol doc's own advice: "Get this into a pair of tested helper functions
 * early. Nearly every map bug traces back to a coordinate conversion."
 */
fun gridToOffset(x: Int, y: Int, gridSize: Int, cellSize: Float): GridOffset =
    GridOffset(
        left = x * cellSize,
        top = (gridSize - 1 - y) * cellSize,
    )

/**
 * The robot's footprint dimensions in grid cells for [direction]: `(sizeX, sizeY)`.
 * The chassis is [lengthCells] long (along the heading axis) by [widthCells] wide
 * (across it), so the box is `widthCells × lengthCells` facing N/S and
 * `lengthCells × widthCells` facing E/W.
 */
fun footprintBox(direction: Direction, lengthCells: Int, widthCells: Int): Pair<Int, Int> =
    when (direction) {
        Direction.NORTH, Direction.SOUTH -> widthCells to lengthCells
        Direction.EAST, Direction.WEST -> lengthCells to widthCells
    }

/**
 * Top-left pixel/dp offset of a robot's footprint, given the rear-center cell it
 * reports itself at ([originX], [originY]) and its current [direction].
 */
fun robotFootprintOffset(
    originX: Int,
    originY: Int,
    direction: Direction,
    gridSize: Int,
    cellSize: Float,
    lengthCells: Int,
    widthCells: Int,
): GridOffset {
    val (_, sizeY) = footprintBox(direction, lengthCells, widthCells)
    // originX/Y is the rear-center of the footprint.
    // Shift it to find the bottom-left visual cell (the true origin of the drawing box)
    val bottomLeftX = when (direction) {
        Direction.NORTH, Direction.SOUTH -> originX - (widthCells / 2)
        Direction.EAST, Direction.WEST -> originX
    }
    val bottomLeftY = when (direction) {
        Direction.NORTH, Direction.SOUTH -> originY
        Direction.EAST, Direction.WEST -> originY - (widthCells / 2)
    }

    return gridToOffset(bottomLeftX, bottomLeftY + sizeY - 1, gridSize, cellSize)
}

/** Integer grid cell coordinates, origin bottom-left, y increasing upward (see [gridToOffset]). */
data class GridCell(val x: Int, val y: Int)

/**
 * Inverse of [gridToOffset]: converts a top-left drawing-space offset back to the grid cell
 * it falls within. Does not clamp — a point outside the grid's drawn extent produces a
 * [GridCell] with x/y outside `0 until gridSize`; callers decide what an out-of-bounds
 * result means (e.g. "dragged off the grid").
 */
fun offsetToGridCell(left: Float, top: Float, gridSize: Int, cellSize: Float): GridCell {
    val x = floor(left / cellSize).toInt()
    val y = gridSize - 1 - floor(top / cellSize).toInt()
    return GridCell(x, y)
}

/**
 * Finds the closest unoccupied, in-bounds cell to [target], searching outward in expanding
 * square rings (Chebyshev distance) so "nearest" matches what a human dragging an obstacle
 * would expect. [target] must already be in bounds (`0 until gridSize` on both axes) —
 * callers resolve "dropped outside the grid" before calling this. Returns null only if
 * every cell in the grid is occupied.
 */
fun nearestFreeCell(target: GridCell, gridSize: Int, isOccupied: (GridCell) -> Boolean): GridCell? {
    if (!isOccupied(target)) return target
    for (radius in 1 until gridSize) {
        for (dx in -radius..radius) {
            for (dy in -radius..radius) {
                if (maxOf(abs(dx), abs(dy)) != radius) continue
                val candidate = GridCell(target.x + dx, target.y + dy)
                if (candidate.x !in 0 until gridSize || candidate.y !in 0 until gridSize) continue
                if (!isOccupied(candidate)) return candidate
            }
        }
    }
    return null
}

/** All grid cells covered by the robot's footprint, given its rear-center origin
 * cell and [direction] — used for obstacle/robot collision checks. */
fun robotFootprintCells(
    originX: Int,
    originY: Int,
    direction: Direction,
    lengthCells: Int,
    widthCells: Int,
): Set<GridCell> {
    val (sizeX, sizeY) = footprintBox(direction, lengthCells, widthCells)
    val bottomLeftX = when (direction) {
        Direction.NORTH, Direction.SOUTH -> originX - (widthCells / 2)
        Direction.EAST, Direction.WEST -> originX
    }
    val bottomLeftY = when (direction) {
        Direction.NORTH, Direction.SOUTH -> originY
        Direction.EAST, Direction.WEST -> originY - (widthCells / 2)
    }
    return (0 until sizeX).flatMap { dx -> (0 until sizeY).map { dy -> GridCell(bottomLeftX + dx, bottomLeftY + dy) } }.toSet()
}

/** Inverse of the box-dimension part of [robotFootprintOffset]: given the grid cell
 * under the footprint's top-left drawing corner, returns the rear-center origin cell
 * the rest of the app tracks the robot by. */
fun originFromTopLeftCell(topLeftCell: GridCell, sizeYCells: Int, direction: Direction, widthCells: Int): GridCell {
    val bottomLeftX = topLeftCell.x
    val bottomLeftY = topLeftCell.y - (sizeYCells - 1)
    
    val originX = when (direction) {
        Direction.NORTH, Direction.SOUTH -> bottomLeftX + (widthCells / 2)
        Direction.EAST, Direction.WEST -> bottomLeftX
    }
    val originY = when (direction) {
        Direction.NORTH, Direction.SOUTH -> bottomLeftY
        Direction.EAST, Direction.WEST -> bottomLeftY + (widthCells / 2)
    }
    return GridCell(originX, originY)
}

/**
 * Robot pose after one of the 4 L-shaped turns (front/back x left/right), replacing
 * pure in-place rotation. Displaces the bottom-left reference cell by a lateral shift
 * of [turnRadiusCells] toward the robot's current right/left, and an equal
 * forward/backward shift of [turnRadiusCells] ahead of/behind its current heading,
 * then rotates heading 90 degrees — see
 * the team design notes. [turnRadiusCells] is
 * the robot's physical turning radius (`ArenaConfig.ROBOT_TURN_RADIUS_CELLS`),
 * independent of its footprint dimensions. Bounds/collision-agnostic; callers check
 * the result before applying it.
 */
fun turnedPose(robot: RobotPose, turn: TurnType, turnRadiusCells: Int): RobotPose {
    val lateralSign = if (turn == TurnType.FRONT_RIGHT || turn == TurnType.BACK_RIGHT) 1 else -1
    val forwardSign = if (turn == TurnType.FRONT_LEFT || turn == TurnType.FRONT_RIGHT) 1 else -1
    val clockwise = turn == TurnType.FRONT_RIGHT || turn == TurnType.BACK_LEFT

    val (rightDx, rightDy) = robot.direction.rotatedClockwise().forwardDelta()
    val (fwdDx, fwdDy) = robot.direction.forwardDelta()

    val dx = lateralSign * turnRadiusCells * rightDx + forwardSign * turnRadiusCells * fwdDx
    val dy = lateralSign * turnRadiusCells * rightDy + forwardSign * turnRadiusCells * fwdDy

    val newDirection =
        if (clockwise) robot.direction.rotatedClockwise() else robot.direction.rotatedCounterClockwise()
    return robot.copy(x = robot.x + dx, y = robot.y + dy, direction = newDirection)
}
