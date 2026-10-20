package com.example.myapplication.arena

/**
 * UI scale multiplier for a given available screen width, relative to the 800dp
 * reference width the mockup was designed at. Clamped to [0.75, 1.35] so scaling
 * never shrinks the 80dp movement buttons below the 48dp minimum touch target, or
 * grows the UI absurdly large on an oversized window.
 */
fun computeUiScale(availableWidthDp: Float, referenceWidthDp: Float = 800f): Float =
    (availableWidthDp / referenceWidthDp).coerceIn(0.75f, 1.35f)
