package com.ntu.mdp.t7.feature.task1_arena

import com.ntu.mdp.t7.core.model.Direction
import com.ntu.mdp.t7.core.model.RobotPose
import com.ntu.mdp.t7.core.model.TurnType
import com.ntu.mdp.t7.core.model.forwardDelta
import com.ntu.mdp.t7.core.model.rotatedClockwise
import com.ntu.mdp.t7.core.model.rotatedCounterClockwise
import kotlin.math.abs
import kotlin.math.floor

data class GridOffset(val left: Float, val top: Float)

fun gridToOffset(x: Int, y: Int, gridSize: Int, cellSize: Float): GridOffset =
    GridOffset(
        left = x * cellSize,
        top = (gridSize - 1 - y) * cellSize,
    )

fun footprintBox(direction: Direction, lengthCells: Int, widthCells: Int): Pair<Int, Int> =
    when (direction) {
        Direction.NORTH, Direction.SOUTH -> widthCells to lengthCells
        Direction.EAST, Direction.WEST -> lengthCells to widthCells
    }

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

data class GridCell(val x: Int, val y: Int)

fun offsetToGridCell(left: Float, top: Float, gridSize: Int, cellSize: Float): GridCell {
    val x = floor(left / cellSize).toInt()
    val y = gridSize - 1 - floor(top / cellSize).toInt()
    return GridCell(x, y)
}

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
