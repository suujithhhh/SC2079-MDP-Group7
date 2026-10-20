package com.example.myapplication.arena

/**
 * Resolves a drop onto [target] to an unoccupied, in-bounds cell, snapping to the nearest
 * free cell on collision (see [nearestFreeCell]). [excludingId], if given, excludes that
 * obstacle from the occupied set by id — not by its current coordinate — so an obstacle
 * already on the grid can be "moved" back onto the cell it currently occupies without
 * falsely colliding with itself, and so a stale/mid-move coordinate can never cause a
 * false exclusion.
 */
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
