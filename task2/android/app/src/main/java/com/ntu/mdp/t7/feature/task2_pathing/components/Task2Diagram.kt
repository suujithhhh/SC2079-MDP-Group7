package com.ntu.mdp.t7.feature.task2_pathing.components

import androidx.compose.animation.core.animateFloatAsState
import androidx.compose.foundation.Canvas
import androidx.compose.foundation.background
import androidx.compose.foundation.border
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.BoxWithConstraints
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.offset
import androidx.compose.foundation.layout.size
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.runtime.Composable
import androidx.compose.runtime.getValue
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.geometry.CornerRadius
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.geometry.Rect
import androidx.compose.ui.geometry.Size
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.Path
import androidx.compose.ui.graphics.StrokeCap
import androidx.compose.ui.graphics.StrokeJoin
import androidx.compose.ui.graphics.drawscope.Stroke
import androidx.compose.ui.graphics.drawscope.withTransform
import androidx.compose.ui.layout.ContentScale
import androidx.compose.ui.platform.LocalContext
import androidx.compose.ui.platform.LocalDensity
import androidx.compose.ui.text.TextStyle
import androidx.compose.ui.text.drawText
import androidx.compose.ui.text.font.FontFamily
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.text.rememberTextMeasurer
import androidx.compose.ui.tooling.preview.Preview
import androidx.compose.ui.unit.IntOffset
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import coil.compose.AsyncImage
import coil.request.ImageRequest
import com.ntu.mdp.t7.core.model.ArrowDir
import com.ntu.mdp.t7.feature.task1_arena.ArenaColors
import com.ntu.mdp.t7.feature.task1_arena.components.PillButton
import com.ntu.mdp.t7.ui.theme.AppTheme
import com.ntu.mdp.t7.ui.theme.MyApplicationTheme
import kotlin.math.roundToInt

@Composable
fun Task2Diagram(
    dist1: Int,
    dist2: Int,
    obs2Length: Int,
    obs1Arrow: ArrowDir?,
    obs2Arrow: ArrowDir?,
    obs1TargetId: Int?,
    obs2TargetId: Int?,
    obs1ImageUrl: String? = null,
    obs2ImageUrl: String? = null,
    robotXCm: Float? = null,
    robotYCm: Float? = null,
    robotHeadingDeg: Float? = null,
    onClearClick: () -> Unit,
    modifier: Modifier = Modifier
) {
    val textMeasurer = rememberTextMeasurer()
    val colors = AppTheme.colors

    val animRobotX by animateFloatAsState(targetValue = robotXCm ?: 0f, label = "robotX")
    val animRobotY by animateFloatAsState(targetValue = robotYCm ?: 0f, label = "robotY")
    val animRobotHeading by animateFloatAsState(targetValue = robotHeadingDeg ?: 0f, label = "robotHeading")

    BoxWithConstraints(
        modifier = modifier
            .fillMaxSize()
            .background(colors.gridBg)
    ) {
        val density = LocalDensity.current
        val widthPx = with(density) { maxWidth.toPx() }
        val heightPx = with(density) { maxHeight.toPx() }

        val wall = 10f
        val innerW = 60f
        val innerH = 60f
        val cpW_cm = wall + innerW
        val cpH_cm = wall + innerH + wall

        val arenaHeightCm = 150f
        val scaleY = heightPx / arenaHeightCm

        val activeWidthCm = cpW_cm + dist1 + 10f + dist2 + 10f

        val dynamicScaleX = widthPx / (activeWidthCm + 40f)
        val scale = minOf(dynamicScaleX, scaleY) * 0.95f

        val offsetX = (widthPx - (activeWidthCm * scale)) / 2f
        val offsetY = (heightPx - (arenaHeightCm * scale)) / 2f

        val cy = offsetY + (arenaHeightCm / 2f) * scale
        val cpX = offsetX
        val cpW = cpW_cm * scale
        val cpH = cpH_cm * scale
        val wallW = wall * scale

        val obs1X = cpX + cpW + (dist1 * scale)
        val obsSize = 10f * scale
        val obs2X = obs1X + obsSize + (dist2 * scale)
        val obs2H = obs2Length * scale

        Canvas(modifier = Modifier.fillMaxSize()) {
            fun drawDimensionArrow(startX: Float, endX: Float, y: Float, label: String, isRed: Boolean = true) {
                val color = if (isRed) Color.Red else colors.arenaBorder
                drawLine(
                    color = color,
                    start = Offset(startX, y),
                    end = Offset(endX, y),
                    strokeWidth = 3.dp.toPx()
                )
                drawLine(color = color, start = Offset(startX, y), end = Offset(startX + 10f, y - 5f), strokeWidth = 3.dp.toPx())
                drawLine(color = color, start = Offset(startX, y), end = Offset(startX + 10f, y + 5f), strokeWidth = 3.dp.toPx())
                drawLine(color = color, start = Offset(endX, y), end = Offset(endX - 10f, y - 5f), strokeWidth = 3.dp.toPx())
                drawLine(color = color, start = Offset(endX, y), end = Offset(endX - 10f, y + 5f), strokeWidth = 3.dp.toPx())

                val textLayoutResult = textMeasurer.measure(label)
                drawText(
                    textLayoutResult = textLayoutResult,
                    color = Color.Black,
                    topLeft = Offset((startX + endX) / 2f - textLayoutResult.size.width / 2f, y - textLayoutResult.size.height - 5f)
                )
            }

            val cpColor = colors.obstacleDefault

            drawRect(color = cpColor, topLeft = Offset(cpX, cy - cpH / 2f), size = Size(cpW, wallW))
            drawRect(color = cpColor, topLeft = Offset(cpX, cy + cpH / 2f - wallW), size = Size(cpW, wallW))
            drawRect(color = cpColor, topLeft = Offset(cpX, cy - cpH / 2f), size = Size(wallW, cpH))

            drawRect(color = Color.Black, topLeft = Offset(cpX, cy - cpH / 2f), size = Size(cpW, cpH), style = Stroke(width = 1.dp.toPx()))

            val bullseyeColor = colors.targetFace
            drawCircle(color = bullseyeColor, radius = 6f, center = Offset(cpX + cpW, cy - cpH / 2f + wallW / 2f))
            drawCircle(color = bullseyeColor, radius = 6f, center = Offset(cpX + cpW, cy + cpH / 2f - wallW / 2f))
            drawCircle(color = bullseyeColor, radius = 6f, center = Offset(cpX + wallW, cy))

            val robotBodyColor = Color(0xFF00796B)
            val robotLegColor = Color(0xFF00564B)
            val robotFrontColor = Color(0xEBFFFFFF)
            val robotTreadColor = Color(0x29FFFFFF)

            val isLiveTrackingActive = robotXCm != null && robotYCm != null
            val rW = 25f * scale
            val rH = 19f * scale

            val startCenterX = cpX + cpW - (rW / 2f)
            val startCenterY = cy

            val liveCenterX = startCenterX + (animRobotX * scale)
            val liveCenterY = startCenterY - (animRobotY * scale)

            val pivot = Offset(liveCenterX, liveCenterY)
            val rotationAngle = if (isLiveTrackingActive) animRobotHeading else 0f

            withTransform({
                rotate(rotationAngle, pivot)
            }) {
                val rX = pivot.x - rW / 2f
                val rY = pivot.y - rH / 2f

                val legW = 4f * scale
                val legH = 5f * scale
                drawRect(color = robotLegColor, topLeft = Offset(rX, rY + 1.5f * scale), size = Size(legW, legH))
                drawRect(color = robotLegColor, topLeft = Offset(rX + rW - legW, rY + 1.5f * scale), size = Size(legW, legH))
                drawRect(color = robotLegColor, topLeft = Offset(rX, rY + rH - legH - 1.5f * scale), size = Size(legW, legH))
                drawRect(color = robotLegColor, topLeft = Offset(rX + rW - legW, rY + rH - legH - 1.5f * scale), size = Size(legW, legH))

                drawRect(color = robotBodyColor, topLeft = Offset(rX + 4f * scale, rY), size = Size(17f * scale, rH))

                val path = Path().apply {
                    moveTo(rX + 21f * scale, rY + rH / 2f - 3.5f * scale)
                    lineTo(rX + 21f * scale + 3.5f * scale, rY + rH / 2f)
                    lineTo(rX + 21f * scale, rY + rH / 2f + 3.5f * scale)
                    close()
                }
                drawPath(path, robotFrontColor)
                drawRect(color = robotFrontColor, topLeft = Offset(rX + 8f * scale, rY + rH / 2f - 1f * scale), size = Size(13f * scale, 2f * scale))
                drawRect(color = robotTreadColor, topLeft = Offset(rX + 6f * scale, rY + 2f * scale), size = Size(12f * scale, 15f * scale))
            }

            fun drawDirectionIndicator(obsX: Float, cy: Float, dir: ArrowDir) {
                val isLeft = dir == ArrowDir.LEFT
                val chevronColor = Color(0xFF6B92D1)
                val badgeColor = Color(0xFF2E384D)

                val s = 8f * scale
                val yOffset = if (isLeft) -35f * scale else 35f * scale
                val center = Offset(obsX, cy + yOffset)
                val badgeYOffset = if (isLeft) -25f * scale else 25f * scale
                val badgeCenter = Offset(obsX, center.y + badgeYOffset)

                val path = Path().apply {
                    if (isLeft) {
                        moveTo(center.x - s, center.y + s)
                        lineTo(center.x, center.y - s)
                        lineTo(center.x + s, center.y + s)

                        moveTo(center.x - s, center.y + s * 2f)
                        lineTo(center.x, center.y)
                        lineTo(center.x + s, center.y + s * 2f)
                    } else {
                        moveTo(center.x - s, center.y - s)
                        lineTo(center.x, center.y + s)
                        lineTo(center.x + s, center.y - s)

                        moveTo(center.x - s, center.y - s * 2f)
                        lineTo(center.x, center.y)
                        lineTo(center.x + s, center.y - s * 2f)
                    }
                }
                drawPath(path, chevronColor, style = Stroke(width = 3.dp.toPx(), cap = StrokeCap.Round, join = StrokeJoin.Round))

                val label = if (isLeft) "LEFT" else "RIGHT"
                val textLayoutResult = textMeasurer.measure(label)
                val badgePaddingX = 8f * scale
                val badgePaddingY = 4f * scale
                val badgeRect = Rect(
                    left = badgeCenter.x - textLayoutResult.size.width / 2f - badgePaddingX,
                    top = badgeCenter.y - textLayoutResult.size.height / 2f - badgePaddingY,
                    right = badgeCenter.x + textLayoutResult.size.width / 2f + badgePaddingX,
                    bottom = badgeCenter.y + textLayoutResult.size.height / 2f + badgePaddingY
                )
                drawRoundRect(
                    color = badgeColor,
                    topLeft = badgeRect.topLeft,
                    size = badgeRect.size,
                    cornerRadius = CornerRadius(4.dp.toPx())
                )
                drawText(
                    textLayoutResult = textLayoutResult,
                    color = Color.White,
                    topLeft = Offset(badgeCenter.x - textLayoutResult.size.width / 2f, badgeCenter.y - textLayoutResult.size.height / 2f)
                )
            }

            val obs1Color = if (obs1TargetId != null) ArenaColors.ObstacleTargetFound else colors.obstacleDefault

            drawRect(color = obs1Color, topLeft = Offset(obs1X, cy - obsSize / 2f), size = Size(obsSize, obsSize))
            drawRect(color = Color.Black, topLeft = Offset(obs1X, cy - obsSize / 2f), size = Size(obsSize, obsSize), style = Stroke(width = 1.dp.toPx()))

            val obs1Label = obs1TargetId?.toString() ?: "1"
            val labelStyle = TextStyle(
                fontSize = 20.sp,
                fontWeight = FontWeight.W900,
                fontFamily = FontFamily.Default,
                color = Color.White
            )
            val textObs1 = textMeasurer.measure(obs1Label, style = labelStyle)
            drawText(
                textLayoutResult = textObs1,
                color = Color.White,
                topLeft = Offset(obs1X + obsSize / 2f - textObs1.size.width / 2f, cy - textObs1.size.height / 2f)
            )

            drawCircle(color = bullseyeColor, radius = 4f, center = Offset(obs1X + obsSize/2f, cy - obsSize / 2f))
            drawCircle(color = bullseyeColor, radius = 4f, center = Offset(obs1X + obsSize/2f, cy + obsSize / 2f))

            if (obs1Arrow != null) {
                drawDirectionIndicator(obs1X + obsSize / 2f, cy, obs1Arrow)
            }

            val obs2Color = if (obs2TargetId != null) ArenaColors.ObstacleTargetFound else colors.obstacleDefault

            drawRect(color = obs2Color, topLeft = Offset(obs2X, cy - obs2H / 2f), size = Size(obsSize, obs2H))
            drawRect(color = Color.Black, topLeft = Offset(obs2X, cy - obs2H / 2f), size = Size(obsSize, obs2H), style = Stroke(width = 1.dp.toPx()))

            val obs2Label = obs2TargetId?.toString() ?: "2"
            val textObs2 = textMeasurer.measure(obs2Label, style = labelStyle)
            drawText(
                textLayoutResult = textObs2,
                color = Color.White,
                topLeft = Offset(obs2X + obsSize / 2f - textObs2.size.width / 2f, cy - textObs2.size.height / 2f)
            )

            drawCircle(color = bullseyeColor, radius = 4f, center = Offset(obs2X + obsSize/2f, cy - obs2H / 2f))
            drawCircle(color = bullseyeColor, radius = 4f, center = Offset(obs2X + obsSize/2f, cy + obs2H / 2f))
            drawCircle(color = bullseyeColor, radius = 4f, center = Offset(obs2X + obsSize, cy))
            drawCircle(color = bullseyeColor, radius = 4f, center = Offset(obs2X, cy - obs2H / 2f + 5f * scale))
            drawCircle(color = bullseyeColor, radius = 4f, center = Offset(obs2X, cy + obs2H / 2f - 5f * scale))
            drawCircle(color = bullseyeColor, radius = 4f, center = Offset(obs2X + obsSize, cy - obs2H / 2f + 5f * scale))
            drawCircle(color = bullseyeColor, radius = 4f, center = Offset(obs2X + obsSize, cy + obs2H / 2f - 5f * scale))

            if (obs2Arrow != null) {
                drawDirectionIndicator(obs2X + obsSize / 2f, cy, obs2Arrow)
            }

            drawDimensionArrow(startX = cpX + cpW, endX = obs1X, y = cy, label = "$dist1 cm")
            drawDimensionArrow(startX = obs1X + obsSize, endX = obs2X, y = cy, label = "$dist2 cm")

            drawDimensionArrow(startX = cpX + wallW, endX = cpX + cpW, y = cy - cpH / 2f - 20f, label = "60 cm", isRed = false)

            drawLine(color = colors.arenaBorder, start = Offset(cpX - 20f, cy - cpH / 2f + wallW), end = Offset(cpX - 20f, cy + cpH / 2f - wallW), strokeWidth = 1.dp.toPx())
            val cpHeightLabel = textMeasurer.measure("60 cm")
            drawText(
                textLayoutResult = cpHeightLabel,
                topLeft = Offset(cpX - 20f - cpHeightLabel.size.width - 5f, cy - cpHeightLabel.size.height / 2f),
                color = Color.Black
            )
        }

        if (obs1ImageUrl != null) {
            val imgSizeDp = 64.dp
            val imgX = (obs1X + obsSize / 2f - with(density) { 32.dp.toPx() }).roundToInt()
            val imgY = (cy - obsSize / 2f - with(density) { 75.dp.toPx() }).roundToInt()
            Box(
                modifier = Modifier.offset { IntOffset(imgX, imgY) }
            ) {
                AsyncImage(
                    model = ImageRequest.Builder(LocalContext.current)
                        .data(obs1ImageUrl)
                        .crossfade(true)
                        .build(),
                    contentDescription = "Obstacle 1 RAW Image",
                    contentScale = ContentScale.Crop,
                    modifier = Modifier
                        .size(imgSizeDp)
                        .clip(RoundedCornerShape(8.dp))
                        .border(2.dp, Color.White, RoundedCornerShape(8.dp))
                )
            }
        }

        if (obs2ImageUrl != null) {
            val imgSizeDp = 64.dp
            val imgX = (obs2X + obsSize / 2f - with(density) { 32.dp.toPx() }).roundToInt()
            val imgY = (cy + obs2H / 2f + with(density) { 15.dp.toPx() }).roundToInt()
            Box(
                modifier = Modifier.offset { IntOffset(imgX, imgY) }
            ) {
                AsyncImage(
                    model = ImageRequest.Builder(LocalContext.current)
                        .data(obs2ImageUrl)
                        .crossfade(true)
                        .build(),
                    contentDescription = "Obstacle 2 RAW Image",
                    contentScale = ContentScale.Crop,
                    modifier = Modifier
                        .size(imgSizeDp)
                        .clip(RoundedCornerShape(8.dp))
                        .border(2.dp, Color.White, RoundedCornerShape(8.dp))
                )
            }
        }

        PillButton(
            label = "CLEAR RUN",
            onClick = onClearClick,
            modifier = Modifier.offset {
                IntOffset(
                    x = cpX.roundToInt(),
                    y = (cy + cpH / 2f + 10f * scale).roundToInt()
                )
            }
        )
    }
}
