package com.example.myapplication.ui.theme

import androidx.compose.material3.ColorScheme
import androidx.compose.material3.darkColorScheme
import androidx.compose.material3.lightColorScheme
import androidx.compose.runtime.staticCompositionLocalOf
import androidx.compose.ui.graphics.Color

// --- Light: Material ColorScheme slots (design-system-brief.md section 4, "Light (default)") ---
private val LightBg = Color(0xFFF4F5F9)
private val LightSurface = Color(0xFFFFFFFF)
private val LightSurfaceSunken = Color(0xFFEDEFF5)
private val LightOutline = Color(0xFFDCE0EA)
private val LightOnSurface = Color(0xFF161A22)
private val LightOnSurfaceMuted = Color(0xFF5C6474)
private val LightPrimary = Color(0xFF4A5AE8)
private val LightPrimaryContainer = Color(0xFFE4E7FF)
private val LightOnPrimary = Color(0xFFFFFFFF)

// --- Dark: Material ColorScheme slots (design-system-brief.md section 4, "Dark") ---
private val DarkBg = Color(0xFF0E1116)
private val DarkSurface = Color(0xFF171B23)
private val DarkSurfaceSunken = Color(0xFF12151C)
private val DarkOutline = Color(0xFF2E3440)
private val DarkOnSurface = Color(0xFFE8EAF0)
private val DarkOnSurfaceMuted = Color(0xFF9AA3B4)
private val DarkPrimary = Color(0xFFA9B4FF)
private val DarkPrimaryContainer = Color(0xFF2B3283)
private val DarkOnPrimary = Color(0xFF10133A)

val AppLightColorScheme: ColorScheme = lightColorScheme(
    primary = LightPrimary,
    onPrimary = LightOnPrimary,
    primaryContainer = LightPrimaryContainer,
    onPrimaryContainer = LightOnSurface,
    // Single-accent design (design-system-brief.md section 10, item 5): secondary/tertiary
    // collapse onto primary rather than inventing new accent hues.
    secondary = LightPrimary,
    onSecondary = LightOnPrimary,
    secondaryContainer = LightPrimaryContainer,
    onSecondaryContainer = LightOnSurface,
    tertiary = LightPrimary,
    onTertiary = LightOnPrimary,
    tertiaryContainer = LightPrimaryContainer,
    onTertiaryContainer = LightOnSurface,
    background = LightBg,
    onBackground = LightOnSurface,
    surface = LightSurface,
    onSurface = LightOnSurface,
    surfaceVariant = LightSurfaceSunken,
    onSurfaceVariant = LightOnSurfaceMuted,
    outline = LightOutline,
    outlineVariant = LightOutline,
    // "surface: panels, cards, sheets" (brief section 4) — this is what the device
    // picker bottom sheet (brief section 8, item 2) should render as.
    surfaceContainerLowest = LightBg,
    surfaceContainerLow = LightSurfaceSunken,
    surfaceContainer = LightSurface,
    surfaceContainerHigh = LightSurface,
    surfaceContainerHighest = LightSurface,
)

val AppDarkColorScheme: ColorScheme = darkColorScheme(
    primary = DarkPrimary,
    onPrimary = DarkOnPrimary,
    primaryContainer = DarkPrimaryContainer,
    onPrimaryContainer = DarkOnSurface,
    // Single-accent design (design-system-brief.md section 10, item 5): secondary/tertiary
    // collapse onto primary rather than inventing new accent hues.
    secondary = DarkPrimary,
    onSecondary = DarkOnPrimary,
    secondaryContainer = DarkPrimaryContainer,
    onSecondaryContainer = DarkOnSurface,
    tertiary = DarkPrimary,
    onTertiary = DarkOnPrimary,
    tertiaryContainer = DarkPrimaryContainer,
    onTertiaryContainer = DarkOnSurface,
    background = DarkBg,
    onBackground = DarkOnSurface,
    surface = DarkSurface,
    onSurface = DarkOnSurface,
    surfaceVariant = DarkSurfaceSunken,
    onSurfaceVariant = DarkOnSurfaceMuted,
    outline = DarkOutline,
    outlineVariant = DarkOutline,
    // "surface: panels, cards, sheets" (brief section 4) — this is what the device
    // picker bottom sheet (brief section 8, item 2) should render as.
    surfaceContainerLowest = DarkBg,
    surfaceContainerLow = DarkSurfaceSunken,
    surfaceContainer = DarkSurface,
    surfaceContainerHigh = DarkSurface,
    surfaceContainerHighest = DarkSurface,
)

/**
 * Tokens with no Material 3 ColorScheme slot: link/obstacle/target-face/robot/grid state
 * colours, per design-system-brief.md section 4. Exposed via [LocalAppColors].
 */
data class AppColors(
    val surfaceSunken: Color,
    val linkIdle: Color,
    val linkWorking: Color,
    val linkOk: Color,
    val linkLost: Color,
    val gridBg: Color,
    val gridLine: Color,
    val gridLineMajor: Color,
    val arenaBorder: Color,
    val obstacleDefault: Color,
    val obstacleIdentified: Color,
    val obstacleRemoval: Color,
    val onObstacle: Color,
    val targetFace: Color,
    val robotBody: Color,
    val onRobot: Color,
)

val LightAppColors = AppColors(
    surfaceSunken = LightSurfaceSunken,
    linkIdle = Color(0xFF6B7280),
    linkWorking = Color(0xFFC07800),
    linkOk = Color(0xFF12855A),
    linkLost = Color(0xFFC5372C),
    gridBg = Color(0xFFFFFFFF),
    gridLine = Color(0xFFE4E7EF),
    gridLineMajor = Color(0xFFC6CBD9),
    arenaBorder = Color(0xFF8F97A8),
    obstacleDefault = Color(0xFF3A4256),
    obstacleIdentified = Color(0xFF1B6E4F),
    obstacleRemoval = Color(0xFFC5372C),
    onObstacle = Color(0xFFFFFFFF),
    targetFace = Color(0xFFFF7A1A),
    robotBody = Color(0xFF00796B),
    onRobot = Color(0xFFFFFFFF),
)

val DarkAppColors = AppColors(
    surfaceSunken = DarkSurfaceSunken,
    linkIdle = Color(0xFF8A93A5),
    linkWorking = Color(0xFFF0B429),
    linkOk = Color(0xFF4ADE80),
    linkLost = Color(0xFFF87171),
    gridBg = Color(0xFF141821),
    gridLine = Color(0xFF232936),
    gridLineMajor = Color(0xFF39404F),
    arenaBorder = Color(0xFF6B7383),
    obstacleDefault = Color(0xFF4C5668),
    obstacleIdentified = Color(0xFF2A8A63),
    obstacleRemoval = Color(0xFFE5534B),
    onObstacle = Color(0xFFFFFFFF),
    targetFace = Color(0xFFFF9445),
    robotBody = Color(0xFF2DD4BF),
    onRobot = Color(0xFF08221F),
)

val LocalAppColors = staticCompositionLocalOf { LightAppColors }
