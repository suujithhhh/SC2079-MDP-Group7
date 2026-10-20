package com.ntu.mdp.t7.feature.task1_arena.components

import androidx.compose.foundation.background
import androidx.compose.foundation.border
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.IntrinsicSize
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.fillMaxHeight
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.width
import androidx.compose.foundation.lazy.LazyColumn
import androidx.compose.foundation.lazy.items
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.text.TextStyle
import androidx.compose.ui.text.font.FontFamily
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import com.ntu.mdp.t7.core.model.StatusLevel
import com.ntu.mdp.t7.core.model.StatusMessage
import com.ntu.mdp.t7.feature.task1_arena.ArenaColors

@Composable
fun StatusPanel(
    entries: List<StatusMessage>,
    modifier: Modifier = Modifier,
) {
    Column(
        modifier = modifier
            .background(ArenaColors.SurfaceWhite, RoundedCornerShape(12.dp))
            .border(1.dp, ArenaColors.Border, RoundedCornerShape(12.dp))
            .padding(16.dp),
        verticalArrangement = Arrangement.spacedBy(12.dp),
    ) {
        Row(modifier = Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.SpaceBetween) {
            Text(text = "Status", style = TextStyle(fontSize = 22.sp, fontWeight = FontWeight.SemiBold, color = ArenaColors.TextPrimary))
            Text(text = "live", style = TextStyle(fontSize = 13.sp, fontFamily = FontFamily.Monospace, color = ArenaColors.TextSecondary))
        }

        LazyColumn(
            modifier = Modifier
                .fillMaxSize()
                .clip(RoundedCornerShape(8.dp))
                .background(ArenaColors.PageBackground)
                .padding(12.dp),
            reverseLayout = true,
            verticalArrangement = Arrangement.spacedBy(8.dp),
        ) {
            items(entries.asReversed()) { entry -> StatusRow(entry) }
        }
    }
}

@Composable
private fun StatusRow(entry: StatusMessage) {
    val borderColor = when (entry.level) {
        StatusLevel.INFO -> ArenaColors.StatusInfoBorder
        StatusLevel.EVENT -> ArenaColors.StatusNoticeBorder
        StatusLevel.ALERT -> ArenaColors.StatusErrorBorder
    }

    Row(
        modifier = Modifier.fillMaxWidth().height(IntrinsicSize.Min),
        horizontalArrangement = Arrangement.spacedBy(12.dp),
    ) {
        Box(
            modifier = Modifier
                .width(3.dp)
                .fillMaxHeight()
                .background(borderColor),
        )
        Text(
            text = entry.time,
            style = TextStyle(fontSize = 13.sp, fontFamily = FontFamily.Monospace, color = ArenaColors.TextSecondary),
        )
        Text(
            text = entry.message,
            style = TextStyle(fontSize = 13.sp, lineHeight = 18.sp, fontFamily = FontFamily.Monospace, color = ArenaColors.TextPrimary),
            modifier = Modifier.weight(1f),
        )
    }
}
