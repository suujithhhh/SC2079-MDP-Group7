package com.ntu.mdp.t7.feature.connection.components

import androidx.compose.foundation.background
import androidx.compose.foundation.border
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.heightIn
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.size
import androidx.compose.foundation.shape.CircleShape
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import com.ntu.mdp.t7.ui.theme.AppType
import com.ntu.mdp.t7.ui.theme.Dimens

@Composable
fun RunTimerCard(
    elapsedMs: Long,
    isRunning: Boolean,
    modifier: Modifier = Modifier,
    title: String = "RUN TIMER",
) {
    val dotColor = if (isRunning) Color(0xFF4CAF50) else Color(0xFFFF9800)
    val statusText = if (isRunning) "RUNNING" else if (elapsedMs > 0) "STOPPED (${formatElapsedTime(elapsedMs)})" else "READY"

    Row(
        modifier = modifier
            .heightIn(min = 72.dp)
            .background(MaterialTheme.colorScheme.surface, RoundedCornerShape(Dimens.RadiusMedium))
            .border(Dimens.BorderWidth, MaterialTheme.colorScheme.outline, RoundedCornerShape(Dimens.RadiusMedium))
            .padding(horizontal = Dimens.Space12),
        verticalAlignment = Alignment.CenterVertically,
        horizontalArrangement = Arrangement.spacedBy(Dimens.Space8),
    ) {
        Box(modifier = Modifier.size(10.dp).background(dotColor, CircleShape))

        Column(
            horizontalAlignment = Alignment.Start,
            verticalArrangement = Arrangement.Center
        ) {
            Text(
                text = title,
                style = AppType.Subtitle,
                fontSize = 10.sp,
                color = MaterialTheme.colorScheme.onSurfaceVariant
            )
            Text(
                text = formatElapsedTime(elapsedMs),
                style = AppType.Mono,
                fontSize = 18.sp,
                fontWeight = FontWeight.Bold,
                color = if (isRunning) MaterialTheme.colorScheme.primary else MaterialTheme.colorScheme.onSurface
            )
            Text(
                text = statusText,
                style = AppType.Mono,
                fontSize = 9.sp,
                color = dotColor
            )
        }
    }
}

fun formatElapsedTime(ms: Long): String {
    val totalSeconds = ms / 1000
    val minutes = totalSeconds / 60
    val seconds = totalSeconds % 60
    val tenths = (ms % 1000) / 100
    return "%02d:%02d.%d".format(minutes, seconds, tenths)
}
