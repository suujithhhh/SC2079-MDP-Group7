package com.ntu.mdp.t7.feature.task1_arena.components

import androidx.compose.foundation.Canvas
import androidx.compose.foundation.background
import androidx.compose.foundation.border
import androidx.compose.foundation.clickable
import androidx.compose.foundation.gestures.detectDragGestures
import androidx.compose.foundation.gestures.detectTapGestures
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.BoxScope
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.offset
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.size
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.rememberUpdatedState
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.draw.drawBehind
import androidx.compose.ui.draw.rotate
import androidx.compose.ui.geometry.CornerRadius
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.geometry.Rect
import androidx.compose.ui.geometry.Size
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.Path
import androidx.compose.ui.graphics.drawscope.Stroke
import androidx.compose.ui.input.pointer.pointerInput
import androidx.compose.ui.layout.boundsInWindow
import androidx.compose.ui.layout.onGloballyPositioned
import androidx.compose.ui.platform.LocalDensity
import androidx.compose.ui.text.TextStyle
import androidx.compose.ui.text.font.FontFamily
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.text.style.TextAlign
import androidx.compose.ui.tooling.preview.Preview
import androidx.compose.ui.unit.Dp
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import com.ntu.mdp.t7.core.model.ArenaConfig
import com.ntu.mdp.t7.core.model.Direction
import com.ntu.mdp.t7.core.model.Obstacle
import com.ntu.mdp.t7.core.model.RobotPose
import com.ntu.mdp.t7.feature.task1_arena.ArenaColors
import com.ntu.mdp.t7.feature.task1_arena.GridCell
import com.ntu.mdp.t7.feature.task1_arena.footprintBox
import com.ntu.mdp.t7.feature.task1_arena.gridToOffset
import com.ntu.mdp.t7.feature.task1_arena.offsetToGridCell
import com.ntu.mdp.t7.feature.task1_arena.originFromTopLeftCell
import com.ntu.mdp.t7.feature.task1_arena.robotFootprintOffset
import com.ntu.mdp.t7.ui.theme.MyApplicationTheme
import kotlin.math.abs

@Composable
fun ArenaGrid(
    obstacles: List<Obstacle>,
    robot: RobotPose,
    modifier: Modifier = Modifier,
    gridSize: Int = ArenaConfig.GRID_SIZE,
    cellSize: Dp = ArenaConfig.CELL_SIZE_DP.dp,
    editing: Boolean = false,
    robotDirectionPickerVisible: Boolean = false,
    highlightedColumn: Int? = null,
    highlightedRow: Int? = null,
    onObstacleClick: (Obstacle) -> Unit = {},
    onGridClick: (GridCell) -> Unit = {},
    onObstaclePlacedWithFace: (GridCell, Direction) -> Unit = { _, _ -> },
    onRobotDragEnd: (GridCell) -> Unit = {},
    onRobotTapped: () -> Unit = {},
    onRobotFaceSelected: (Direction) -> Unit = {},
    onGridBoundsChanged: (Rect) -> Unit = {},
    onColumnLabelClick: (Int) -> Unit = {},
    onRowLabelClick: (Int) -> Unit = {},
) {
    val gridExtent = cellSize * gridSize

    Row(
        modifier = modifier
            .background(ArenaColors.SurfaceWhite, RoundedCornerShape(20.dp))
            .border(1.dp, ArenaColors.Border, RoundedCornerShape(20.dp))
            .padding(12.dp),
    ) {
        RowLabels(
            gridSize = gridSize, 
            cellSize = cellSize,
            onRowLabelClick = onRowLabelClick
        )

            Column {
                Box(
                    modifier = Modifier
                        .size(gridExtent)
                        .onGloballyPositioned { coordinates -> onGridBoundsChanged(coordinates.boundsInWindow()) }
                        .then(
                            if (editing) {
                                Modifier
                                    .pointerInput(editing, highlightedColumn, highlightedRow) {
                                        detectTapGestures(
                                            onTap = { offset ->
                                                var cell = offsetToGridCell(offset.x, offset.y, gridSize, cellSize.toPx())
                                                if (highlightedColumn != null && abs(cell.x - highlightedColumn) <= 1) {
                                                    cell = cell.copy(x = highlightedColumn)
                                                }
                                                if (highlightedRow != null && abs(cell.y - highlightedRow) <= 1) {
                                                    cell = cell.copy(y = highlightedRow)
                                                }
                                                onGridClick(cell)
                                            }
                                        )
                                    }
                                    .pointerInput(editing, highlightedColumn, highlightedRow) {
                                        var dragOrigin = Offset.Zero
                                        var totalDragX = 0f
                                        var totalDragY = 0f
                                        detectDragGestures(
                                            onDragStart = { offset ->
                                                dragOrigin = offset
                                                totalDragX = 0f
                                                totalDragY = 0f
                                            },
                                            onDrag = { change, dragAmount ->
                                                change.consume()
                                                totalDragX += dragAmount.x
                                                totalDragY += dragAmount.y
                                            },
                                            onDragEnd = {
                                                var cell = offsetToGridCell(dragOrigin.x, dragOrigin.y, gridSize, cellSize.toPx())
                                                if (highlightedColumn != null && abs(cell.x - highlightedColumn) <= 1) {
                                                    cell = cell.copy(x = highlightedColumn)
                                                }
                                                if (highlightedRow != null && abs(cell.y - highlightedRow) <= 1) {
                                                    cell = cell.copy(y = highlightedRow)
                                                }
                                                val face = if (abs(totalDragX) > abs(totalDragY)) {
                                                    if (totalDragX > 0) Direction.EAST else Direction.WEST
                                                } else {
                                                    if (totalDragY < 0) Direction.NORTH else Direction.SOUTH
                                                }
                                                onObstaclePlacedWithFace(cell, face)
                                            }
                                        )
                                    }
                            } else Modifier
                        ),
                ) {
                    GridBackground(
                        gridSize = gridSize, 
                        cellSize = cellSize,
                        highlightedColumn = highlightedColumn,
                        highlightedRow = highlightedRow,
                        modifier = Modifier.size(gridExtent)
                    )

                    obstacles.forEach { obstacle ->
                        ObstacleMarker(
                            obstacle = obstacle,
                            gridSize = gridSize,
                            cellSize = cellSize,
                            onClick = { onObstacleClick(obstacle) },
                        )
                    }

                    RobotMarker(
                        robot = robot,
                        gridSize = gridSize,
                        cellSize = cellSize,
                        editing = editing,
                        onDragEnd = onRobotDragEnd,
                        onTapped = onRobotTapped,
                    )

                    if (robotDirectionPickerVisible) {
                        RobotDirectionButtons(
                            robot = robot,
                            gridSize = gridSize,
                            cellSize = cellSize,
                            onFaceSelected = onRobotFaceSelected,
                        )
                    }
                }

                ColumnLabels(
                gridSize = gridSize, 
                cellSize = cellSize,
                onColumnLabelClick = onColumnLabelClick
            )
        }
    }
}

@Composable
private fun RowLabels(gridSize: Int, cellSize: Dp, onRowLabelClick: (Int) -> Unit = {}) {
    Column(horizontalAlignment = Alignment.End) {
        for (row in (gridSize - 1) downTo 0) {
            Box(
                modifier = Modifier
                    .size(width = 16.dp, height = cellSize)
                    .clickable { onRowLabelClick(row) }, 
                contentAlignment = Alignment.Center
            ) {
                Text(text = row.toString(), style = labelStyle)
            }
        }
    }
}

@Composable
private fun ColumnLabels(gridSize: Int, cellSize: Dp, onColumnLabelClick: (Int) -> Unit = {}) {
    Row {
        for (col in 0 until gridSize) {
            Box(
                modifier = Modifier
                    .size(cellSize)
                    .clickable { onColumnLabelClick(col) }, 
                contentAlignment = Alignment.Center
            ) {
                Text(text = col.toString(), style = labelStyle)
            }
        }
    }
}

internal val ObstacleLabelFontSize = 16.5.sp
internal val ObstacleOutlineWidth = 1.5.dp

private val labelStyle = TextStyle(
    fontSize = 11.sp,
    fontWeight = FontWeight.Medium,
    fontFamily = FontFamily.Monospace,
    color = ArenaColors.TextSecondary,
)

@Composable
private fun GridBackground(
    gridSize: Int, 
    cellSize: Dp, 
    highlightedColumn: Int? = null,
    highlightedRow: Int? = null,
    modifier: Modifier = Modifier
) {
    Canvas(modifier = modifier.border(2.dp, ArenaColors.GridBorder)) {
        val cellPx = cellSize.toPx()
        val extentPx = cellPx * gridSize
        
        val highlightColor = Color(0xFFFFFF00).copy(alpha = 0.3f)
        if (highlightedColumn != null && highlightedColumn in 0 until gridSize) {
            drawRect(
                color = highlightColor,
                topLeft = Offset(highlightedColumn * cellPx, 0f),
                size = Size(cellPx, extentPx)
            )
        }
        if (highlightedRow != null && highlightedRow in 0 until gridSize) {
            val canvasRow = gridSize - 1 - highlightedRow
            drawRect(
                color = highlightColor,
                topLeft = Offset(0f, canvasRow * cellPx),
                size = Size(extentPx, cellPx)
            )
        }

        for (i in 0..gridSize) {
            val isMajor = i % 5 == 0
            val color = if (isMajor) ArenaColors.GridLineMajor else ArenaColors.GridLineMinor
            val strokeWidth = if (isMajor) 1.5.dp.toPx() else 1.dp.toPx()
            drawLine(color, Offset(i * cellPx, 0f), Offset(i * cellPx, extentPx), strokeWidth)
            drawLine(color, Offset(0f, i * cellPx), Offset(extentPx, i * cellPx), strokeWidth)
        }
    }
}

@Composable
private fun ObstacleMarker(
    obstacle: Obstacle,
    gridSize: Int,
    cellSize: Dp,
    onClick: () -> Unit,
) {
    val baseOffset = gridToOffset(obstacle.x, obstacle.y, gridSize, cellSize.value)
    val inset = (cellSize.value - ArenaConfig.OBSTACLE_SIZE_DP) / 2f
    val identified = obstacle.targetId != null
    val fill = if (identified) ArenaColors.ObstacleTargetFound else ArenaColors.ObstacleUnrecognized

    Box(
        modifier = Modifier
            .offset(
                x = (baseOffset.left + inset).dp,
                y = (baseOffset.top + inset).dp,
            )
            .size(ArenaConfig.OBSTACLE_SIZE_DP.dp)
            .then(if (obstacle.isSelected) Modifier.selectionRing() else Modifier)
            .clip(RoundedCornerShape(6.dp))
            .background(fill)
            .then(
                if (obstacle.isSelected) {
                    Modifier
                } else {
                    Modifier.border(
                        if (identified) 2.dp else ObstacleOutlineWidth,
                        if (identified) Color.White else ArenaColors.ObstacleOutlineDefault,
                        RoundedCornerShape(6.dp)
                    )
                },
            )
            .clickable(onClick = onClick),
        contentAlignment = Alignment.Center,
    ) {
        Column(
            horizontalAlignment = Alignment.CenterHorizontally,
            verticalArrangement = Arrangement.Center
        ) {
            if (identified) {
                Text(
                    text = obstacle.targetId.toString(),
                    style = TextStyle(
                        fontSize = 20.sp,
                        fontWeight = FontWeight.W900,
                        color = Color.White,
                        textAlign = TextAlign.Center,
                    ),
                )
                Text(
                    text = "B${obstacle.id}",
                    style = TextStyle(
                        fontSize = 10.sp,
                        fontWeight = FontWeight.W900,
                        color = Color.White.copy(alpha = 0.95f),
                        textAlign = TextAlign.Center,
                    ),
                )
            } else {
                Text(
                    text = obstacle.id.toString(),
                    style = TextStyle(
                        fontSize = 22.sp,
                        fontWeight = FontWeight.W900,
                        color = ArenaColors.SurfaceWhite,
                        textAlign = TextAlign.Center,
                    ),
                )
            }
        }

        obstacle.face?.let { face -> FaceStrip(face) }
    }
}

private fun Modifier.selectionRing(): Modifier = drawBehind {
    val ringWidth = 3.dp.toPx()
    val glowWidth = 3.dp.toPx()
    val baseCorner = 6.dp.toPx()

    fun ring(outset: Float, strokeWidth: Float, color: Color) {
        drawRoundRect(
            color = color,
            topLeft = Offset(-outset, -outset),
            size = Size(size.width + 2 * outset, size.height + 2 * outset),
            cornerRadius = CornerRadius(baseCorner + outset),
            style = Stroke(width = strokeWidth),
        )
    }

    ring(ringWidth + glowWidth / 2f, glowWidth, ArenaColors.AccentIndigoSoft)
    ring(ringWidth / 2f, ringWidth, ArenaColors.AccentIndigo)
}

@Composable
private fun BoxScope.FaceStrip(face: Direction) {
    val size = ArenaConfig.OBSTACLE_SIZE_DP.dp
    val (stripModifier, alignment) = when (face) {
        Direction.NORTH -> Modifier.size(width = size, height = 5.dp) to Alignment.TopCenter
        Direction.SOUTH -> Modifier.size(width = size, height = 5.dp) to Alignment.BottomCenter
        Direction.EAST -> Modifier.size(width = 5.dp, height = size) to Alignment.CenterEnd
        Direction.WEST -> Modifier.size(width = 5.dp, height = size) to Alignment.CenterStart
    }
    Box(modifier = stripModifier.align(alignment).background(ArenaColors.ObstacleFace))
}

@Composable
private fun RobotMarker(
    robot: RobotPose,
    gridSize: Int,
    cellSize: Dp,
    editing: Boolean,
    onDragEnd: (GridCell) -> Unit,
    onTapped: () -> Unit,
) {
    val lengthCells = ArenaConfig.ROBOT_FOOTPRINT_LENGTH
    val widthCells = ArenaConfig.ROBOT_FOOTPRINT_WIDTH
    val (sizeXCells, sizeYCells) = footprintBox(robot.direction, lengthCells, widthCells)
    val baseOffset = robotFootprintOffset(robot.x, robot.y, robot.direction, gridSize, cellSize.value, lengthCells, widthCells)
    val currentBaseOffset by rememberUpdatedState(baseOffset)
    val currentOnDragEnd by rememberUpdatedState(onDragEnd)
    val currentOnTapped by rememberUpdatedState(onTapped)
    val footprintWidth = cellSize * sizeXCells
    val footprintHeight = cellSize * sizeYCells
    val scaleX = (widthCells.toFloat() / 3f) * (cellSize.value / ArenaConfig.CELL_SIZE_DP)
    val scaleY = (lengthCells.toFloat() / 3f) * (cellSize.value / ArenaConfig.CELL_SIZE_DP)
    fun sx(designDp: Int) = (designDp * scaleX).dp
    fun sy(designDp: Int) = (designDp * scaleY).dp
    val density = LocalDensity.current.density

    var dragOffsetDp by remember(robot.x, robot.y) { mutableStateOf(Offset.Zero) }

    val rotation = when (robot.direction) {
        Direction.NORTH -> 0f
        Direction.EAST -> 90f
        Direction.SOUTH -> 180f
        Direction.WEST -> 270f
    }

    Box(
        contentAlignment = Alignment.Center,
        modifier = Modifier
            .offset(
                x = (baseOffset.left + dragOffsetDp.x).dp,
                y = (baseOffset.top + dragOffsetDp.y).dp,
            )
            .size(width = footprintWidth, height = footprintHeight)
            .then(
                if (editing) {
                    Modifier.pointerInput(robot.x, robot.y) {
                        detectDragGestures(
                            onDragEnd = {
                                val dropTopLeftX = currentBaseOffset.left + dragOffsetDp.x
                                val dropTopLeftY = currentBaseOffset.top + dragOffsetDp.y
                                val topLeftCell =
                                    offsetToGridCell(dropTopLeftX, dropTopLeftY, gridSize, cellSize.value)
                                currentOnDragEnd(originFromTopLeftCell(topLeftCell, sizeYCells, robot.direction, widthCells))
                                dragOffsetDp = Offset.Zero
                            },
                            onDragCancel = { dragOffsetDp = Offset.Zero },
                            onDrag = { change, dragAmount ->
                                change.consume()
                                dragOffsetDp += dragAmount / density
                            },
                        )
                    }
                } else {
                    Modifier
                },
            )
            .then(
                if (editing) {
                    Modifier.pointerInput(robot.x, robot.y) {
                        detectTapGestures(onTap = { currentOnTapped() })
                    }
                } else {
                    Modifier
                },
            ),
    ) {
        Box(modifier = Modifier.size(width = sx(108), height = sy(108)).rotate(rotation)) {
            RobotLeg(Modifier.offset(x = sx(0), y = sy(10)).size(width = sx(18), height = sy(36)))
            RobotLeg(Modifier.offset(x = sx(90), y = sy(10)).size(width = sx(18), height = sy(36)))
            RobotLeg(Modifier.offset(x = sx(0), y = sy(62)).size(width = sx(18), height = sy(36)))
            RobotLeg(Modifier.offset(x = sx(90), y = sy(62)).size(width = sx(18), height = sy(36)))

            Box(
                modifier = Modifier
                    .offset(x = sx(18), y = sy(0))
                    .size(width = sx(72), height = sy(108))
                    .clip(RoundedCornerShape(10.dp))
                    .background(ArenaColors.RobotBody),
            ) {
                Canvas(modifier = Modifier.offset(x = sx(24), y = sy(0)).size(width = sx(24), height = sy(11))) {
                    val path = Path().apply {
                        moveTo(0f, 0f)
                        lineTo(size.width, 0f)
                        lineTo(size.width / 2f, size.height)
                        close()
                    }
                    drawPath(path, ArenaColors.RobotFrontMarker)
                }
                Box(
                    modifier = Modifier
                        .offset(x = sx(33), y = sy(14))
                        .size(width = sx(5), height = sy(26))
                        .clip(RoundedCornerShape(3.dp))
                        .background(ArenaColors.RobotFrontMarker),
                )
                Box(
                    modifier = Modifier
                        .offset(x = sx(12), y = sy(52))
                        .size(width = sx(48), height = sy(32))
                        .clip(RoundedCornerShape(5.dp))
                        .background(ArenaColors.RobotTreadHighlight),
                )
            }
        }
    }
}

@Composable
private fun RobotDirectionButtons(
    robot: RobotPose,
    gridSize: Int,
    cellSize: Dp,
    onFaceSelected: (Direction) -> Unit,
) {
    val lengthCells = ArenaConfig.ROBOT_FOOTPRINT_LENGTH
    val widthCells = ArenaConfig.ROBOT_FOOTPRINT_WIDTH
    val (sizeXCells, sizeYCells) = footprintBox(robot.direction, lengthCells, widthCells)
    val baseOffset =
        robotFootprintOffset(robot.x, robot.y, robot.direction, gridSize, cellSize.value, lengthCells, widthCells)
    val footprintWidth = cellSize * sizeXCells
    val footprintHeight = cellSize * sizeYCells

    val northX = baseOffset.left.dp + (footprintWidth - cellSize) / 2
    val northY = baseOffset.top.dp - cellSize
    val southY = baseOffset.top.dp + footprintHeight
    val westX = baseOffset.left.dp - cellSize
    val westY = baseOffset.top.dp + (footprintHeight - cellSize) / 2
    val eastX = baseOffset.left.dp + footprintWidth

    Box(modifier = Modifier.offset(x = northX, y = northY)) {
        FaceButton(Direction.NORTH, robot.direction, onFaceSelected, cellSize)
    }
    Box(modifier = Modifier.offset(x = northX, y = southY)) {
        FaceButton(Direction.SOUTH, robot.direction, onFaceSelected, cellSize)
    }
    Box(modifier = Modifier.offset(x = westX, y = westY)) {
        FaceButton(Direction.WEST, robot.direction, onFaceSelected, cellSize)
    }
    Box(modifier = Modifier.offset(x = eastX, y = westY)) {
        FaceButton(Direction.EAST, robot.direction, onFaceSelected, cellSize)
    }
}

@Composable
private fun RobotLeg(modifier: Modifier) {
    Box(modifier = modifier.clip(RoundedCornerShape(4.dp)).background(ArenaColors.RobotLeg))
}

@Preview(name = "ArenaGrid - robot headings", widthDp = 500, heightDp = 2100)
@Composable
private fun RobotHeadingsPreview() {
    MyApplicationTheme(darkTheme = false) {
        Column {
            listOf(Direction.NORTH, Direction.EAST, Direction.SOUTH, Direction.WEST).forEach { direction ->
                ArenaGrid(
                    obstacles = emptyList(),
                    robot = RobotPose(x = 5, y = 5, direction = direction),
                    gridSize = 12,
                    modifier = Modifier.padding(bottom = 8.dp),
                )
            }
        }
    }
}
