package com.example.myapplication.ui.theme

import androidx.compose.ui.unit.dp

/** Spacing/shape/touch-target tokens. Per docs/design/design-system-brief.md section 6. */
object Dimens {
    val Space4 = 4.dp
    val Space8 = 8.dp
    val Space12 = 12.dp
    val Space16 = 16.dp
    val Space24 = 24.dp
    val Space32 = 32.dp

    val RadiusSmall = 8.dp
    val RadiusMedium = 12.dp
    val RadiusLarge = 20.dp
    val RadiusPill = 999.dp

    val TouchTargetMin = 48.dp
    val MovementPadButton = 80.dp

    val BorderWidth = 1.dp

    /** Motion durations, ms. Per docs/design/design-system-brief.md section 6. */
    const val StateChangeDurationMs = 150
    const val InspectorSlideDurationMs = 200
    const val RobotMoveDurationMs = 180
}
