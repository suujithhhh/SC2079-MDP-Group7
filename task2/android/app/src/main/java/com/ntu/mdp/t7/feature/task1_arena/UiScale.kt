package com.ntu.mdp.t7.feature.task1_arena

fun computeUiScale(availableWidthDp: Float, availableHeightDp: Float = 1200f, referenceWidthDp: Float = 800f, referenceHeightDp: Float = 1200f): Float {
    val scaleX = availableWidthDp / referenceWidthDp
    val scaleY = availableHeightDp / referenceHeightDp
    return minOf(scaleX, scaleY).coerceIn(0.5f, 1.35f)
}
