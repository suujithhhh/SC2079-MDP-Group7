package com.example.myapplication.ui.theme

import androidx.compose.ui.unit.dp

/**
 * Canvas tokens for the arena map. Per docs/design/design-system-brief.md section 7.
 * CellSize is fixed at 36dp to match the shipped mockup and ArenaGeometryTest exactly,
 * rather than the brief's own approximate derivation formula.
 */
object MapTokens {
    const val GRID_SIZE = 20
    val ArenaPad = 12.dp
    val CellSize = 36.dp
    val GridLineWeight = 1.dp
    val GridLineMajorWeight = 1.5.dp
    val ArenaBorderWeight = 2.dp
    val ObstacleInset = 2.dp
    val ObstacleRadius = 6.dp
    val TargetFaceWeight = 5.dp
    val RobotRadius = 10.dp
    const val HeadingNotchFraction = 0.4f
    const val DragGhostOpacity = 0.55f
    val DragShadow = 8.dp
    const val InvalidDropTintAlpha = 0.18f
}
