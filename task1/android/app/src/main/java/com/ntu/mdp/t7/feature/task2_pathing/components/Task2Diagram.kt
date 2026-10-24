package com.ntu.mdp.t7.feature.task2_pathing.components

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
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.geometry.CornerRadius
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.geometry.Rect
import androidx.compose.ui.geometry.Size
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.Path
import androidx.compose.ui.graphics.PathEffect
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
import androidx.compose.ui.unit.IntOffset
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import androidx.compose.material3.Text
import androidx.compose.ui.Alignment
import androidx.compose.foundation.layout.padding
import coil.compose.AsyncImage
import coil.request.ImageRequest
import com.ntu.mdp.t7.core.model.ArrowDir
import com.ntu.mdp.t7.feature.task1_arena.ArenaColors
import com.ntu.mdp.t7.feature.task1_arena.components.PillButton
import com.ntu.mdp.t7.feature.task2_pathing.Task2RobotStep
import com.ntu.mdp.t7.ui.theme.AppTheme
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
    robotStep: Task2RobotStep,
    onClearClick: () -> Unit,
    modifier: Modifier = Modifier,
    obs1ImageUrl: String? = null,
    obs2ImageUrl: String? = null,
) {
    val textMeasurer = rememberTextMeasurer()
    val colors = AppTheme.colors

    BoxWithConstraints(
        modifier = modifier
            .fillMaxSize()
            .background(colors.gridBg),
    ) {
        val density = LocalDensity.current
        val widthPx = with(density) { maxWidth.toPx() }
        val heightPx = with(density) { maxHeight.toPx() }

        val wall = 10f
        val innerW = 60f
        val innerH = 60f
        val cpWCm = wall + innerW
        val cpHCm = wall + innerH + wall

        val arenaHeightCm = cpHCm + 40f
        
        // Use fixed visual distances so the overall scale and positions don't change
        // when the real dist1 and dist2 values change.
        val visualDist1 = 50f
        val visualDist2 = 50f
        val activeWidthCm = cpWCm + visualDist1 + 10f + visualDist2 + 10f + 40f

        val scaleX = widthPx / activeWidthCm
        val scaleY = heightPx / arenaHeightCm
        val scale = minOf(scaleX, scaleY)

        val cpX = ((widthPx - (activeWidthCm * scale)) / 2f) + (20f * scale)
        val offsetY = ((heightPx - (arenaHeightCm * scale)) / 2f) + (20f * scale)

        val cy = offsetY + ((cpHCm / 2f) * scale)
        val cpW = cpWCm * scale
        val cpH = cpHCm * scale
        val wallW = wall * scale

        val obs1X = cpX + cpW + (visualDist1 * scale)
        val obsSize = 10f * scale
        val obs2X = obs1X + obsSize + (visualDist2 * scale)
        val obs2H = obs2Length * scale

        // Legend: Car Park inner dimension
        Text(
            text = "Car Park: 60x60 cm",
            style = TextStyle(
                fontFamily = FontFamily.Default,
                fontWeight = FontWeight.Medium,
                fontSize = 12.sp,
                color = colors.arenaBorder
            ),
            modifier = Modifier
                .align(Alignment.TopStart)
                .padding(16.dp)
        )

        Canvas(modifier = Modifier.fillMaxSize()) {
            fun drawDimensionBadge(startX: Float, endX: Float, y: Float, label: String) {
                drawLine(
                    color = Color.Gray,
                    start = Offset(startX, y),
                    end = Offset(endX, y),
                    strokeWidth = 1.5.dp.toPx(),
                    pathEffect = PathEffect.dashPathEffect(floatArrayOf(15f, 15f), 0f)
                )

                val textStyle = TextStyle(
                    fontSize = 12.sp,
                    fontWeight = FontWeight.Bold,
                    color = Color.DarkGray
                )
                val textLayoutResult = textMeasurer.measure(label, style = textStyle)
                
                val textWidth = textLayoutResult.size.width
                val textHeight = textLayoutResult.size.height
                val centerX = (startX + endX) / 2f
                
                val horizontalPadding = 24f
                val verticalPadding = 12f
                val badgeWidth = textWidth + horizontalPadding
                val badgeHeight = textHeight + verticalPadding
                
                val badgeRect = Rect(
                    offset = Offset(centerX - badgeWidth / 2f, y - badgeHeight / 2f),
                    size = Size(badgeWidth, badgeHeight)
                )

                drawRoundRect(
                    color = Color.Black.copy(alpha = 0.15f),
                    topLeft = badgeRect.topLeft + Offset(2f, 4f),
                    size = badgeRect.size,
                    cornerRadius = CornerRadius(badgeHeight / 2f)
                )

                drawRoundRect(
                    color = Color.White,
                    topLeft = badgeRect.topLeft,
                    size = badgeRect.size,
                    cornerRadius = CornerRadius(badgeHeight / 2f)
                )

                drawRoundRect(
                    color = Color.LightGray,
                    topLeft = badgeRect.topLeft,
                    size = badgeRect.size,
                    cornerRadius = CornerRadius(badgeHeight / 2f),
                    style = Stroke(width = 1.dp.toPx())
                )

                drawText(
                    textLayoutResult = textLayoutResult,
                    color = Color.DarkGray,
                    topLeft = Offset(centerX - textWidth / 2f, y - textHeight / 2f)
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

            val rW = 25f * scale
            val rH = 19f * scale

            val startCenterX = cpX + cpW - (rW / 2f)
            val startCenterY = cy

            val (liveCenterX, liveCenterY, rotationAngle) = when (robotStep) {
                Task2RobotStep.START -> Triple(startCenterX, startCenterY, 0f)
                Task2RobotStep.AT_OBS_1 -> Triple(obs1X - (30f * scale), cy, 0f)
                Task2RobotStep.AT_OBS_2 -> Triple(obs2X - (30f * scale), cy, 0f)
                Task2RobotStep.END -> Triple(cpX + (cpW / 2f), cy, 180f)
            }

            val pivot = Offset(liveCenterX, liveCenterY)

            withTransform(
                {
                    rotate(rotationAngle, pivot)
                },
            ) {
                val rX = pivot.x - rW / 2f
                val rY = pivot.y - rH / 2f

                val legW = 4f * scale
                val legH = 5f * scale

                drawRect(color = robotLegColor, topLeft = Offset(rX, rY + 1.5f * scale), size = Size(legW, legH))
                drawRect(color = robotLegColor, topLeft = Offset(rX + rW - legW, rY + 1.5f * scale), size = Size(legW, legH))
                drawRect(color = robotLegColor, topLeft = Offset(rX, rY + rH - legH - (1.5f * scale)), size = Size(legW, legH))
                drawRect(color = robotLegColor, topLeft = Offset(rX + rW - legW, rY + rH - legH - (1.5f * scale)), size = Size(legW, legH))

                drawRect(color = robotBodyColor, topLeft = Offset(rX + 4f * scale, rY), size = Size(17f * scale, rH))

                val path = Path().apply {
                    moveTo(rX + 21f * scale, rY + rH / 2f - 3.5f * scale)
                    lineTo(rX + 21f * scale + 3.5f * scale, rY + rH / 2f)
                    lineTo(rX + 21f * scale, rY + rH / 2f + 3.5f * scale)
                    close()
                }
                drawPath(path, robotFrontColor)
                drawRect(color = robotFrontColor, topLeft = Offset(rX + 8f * scale, rY + (rH / 2f) - (1f * scale)), size = Size(13f * scale, 2f * scale))
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
                    left = badgeCenter.x - (textLayoutResult.size.width / 2f) - badgePaddingX,
                    top = badgeCenter.y - (textLayoutResult.size.height / 2f) - badgePaddingY,
                    right = badgeCenter.x + (textLayoutResult.size.width / 2f) + badgePaddingX,
                    bottom = badgeCenter.y + (textLayoutResult.size.height / 2f) + badgePaddingY
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
                    topLeft = Offset(badgeCenter.x - textLayoutResult.size.width / 2f, badgeCenter.y - textLayoutResult.size.height / 2f),
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
                topLeft = Offset(obs1X + obsSize / 2f - textObs1.size.width / 2f, cy - (textObs1.size.height / 2f))
            )

            drawCircle(color = bullseyeColor, radius = 4f, center = Offset(obs1X + obsSize/2f, cy - obsSize / 2f))
            drawCircle(color = bullseyeColor, radius = 4f, center = Offset(obs1X + obsSize/2f, cy + obsSize / 2f))

            obs1Arrow?.let {
                drawDirectionIndicator(obs1X + obsSize / 2f, cy, it)
            }

            val obs2Color = if (obs2TargetId != null) ArenaColors.ObstacleTargetFound else colors.obstacleDefault

            drawRect(color = obs2Color, topLeft = Offset(obs2X, cy - obs2H / 2f), size = Size(obsSize, obs2H))
            drawRect(color = Color.Black, topLeft = Offset(obs2X, cy - obs2H / 2f), size = Size(obsSize, obs2H), style = Stroke(width = 1.dp.toPx()))

            val obs2Label = obs2TargetId?.toString() ?: "2"
            val textObs2 = textMeasurer.measure(obs2Label, style = labelStyle)
            drawText(
                textLayoutResult = textObs2,
                color = Color.White,
                topLeft = Offset(obs2X + obsSize / 2f - textObs2.size.width / 2f, cy - (textObs2.size.height / 2f)),
            )

            drawCircle(color = bullseyeColor, radius = 4f, center = Offset(obs2X + obsSize/2f, cy - obs2H / 2f))
            drawCircle(color = bullseyeColor, radius = 4f, center = Offset(obs2X + obsSize/2f, cy + obs2H / 2f))
            drawCircle(color = bullseyeColor, radius = 4f, center = Offset(obs2X + obsSize, cy))
            drawCircle(color = bullseyeColor, radius = 4f, center = Offset(obs2X, cy - (obs2H / 2f) + (5f * scale)))
            drawCircle(color = bullseyeColor, radius = 4f, center = Offset(obs2X, cy + (obs2H / 2f) - (5f * scale)))
            drawCircle(color = bullseyeColor, radius = 4f, center = Offset(obs2X + obsSize, cy - (obs2H / 2f) + (5f * scale)))
            drawCircle(color = bullseyeColor, radius = 4f, center = Offset(obs2X + obsSize, cy + (obs2H / 2f) - (5f * scale)))

            obs2Arrow?.let {
                drawDirectionIndicator(obs2X + obsSize / 2f, cy, it)
            }

            drawDimensionBadge(startX = cpX + cpW, endX = obs1X, y = cy, label = "$dist1 cm")
            drawDimensionBadge(startX = obs1X + obsSize, endX = obs2X, y = cy, label = "$dist2 cm")
        }

        if (obs1ImageUrl != null) {
            val imgSizeDp = 64.dp
            val imgX = (obs1X + obsSize / 2f - with(density) { 32.dp.toPx() }).roundToInt()
            val imgY = (cy - obsSize / 2f - with(density) { 75.dp.toPx() }).roundToInt()
            Box(
                modifier = Modifier.offset { IntOffset(imgX, imgY) },
            ) {
                AsyncImage(
                    model = ImageRequest.Builder(LocalContext.current)
                        .data(obs1ImageUrl)
                        .crossfade(enable = true)
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
                modifier = Modifier.offset { IntOffset(imgX, imgY) },
            ) {
                AsyncImage(
                    model = ImageRequest.Builder(LocalContext.current)
                        .data(obs2ImageUrl)
                        .crossfade(enable = true)
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
                    y = (cy + cpH / 2f + 10f * scale).roundToInt(),
                )
            },
        )
    }
}
