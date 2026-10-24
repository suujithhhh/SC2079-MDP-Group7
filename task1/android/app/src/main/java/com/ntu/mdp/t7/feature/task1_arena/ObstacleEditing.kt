package com.ntu.mdp.t7.feature.task1_arena

import com.ntu.mdp.t7.core.model.Obstacle

fun resolveDropCell(
    obstacles: List<Obstacle>,
    excludingId: Int?,
    robotFootprint: Set<GridCell>,
    gridSize: Int,
    target: GridCell,
): GridCell {
    val occupied = obstacles.asSequence()
        .filter { it.id != excludingId }
        .map { GridCell(it.x, it.y) }
        .toSet() + robotFootprint
    val clamped = GridCell(target.x.coerceIn(0, gridSize - 1), target.y.coerceIn(0, gridSize - 1))
    return nearestFreeCell(clamped, gridSize) { it in occupied } ?: clamped
}
