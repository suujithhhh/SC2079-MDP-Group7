package com.ntu.mdp.t7.feature.connection.components

import androidx.compose.foundation.background
import androidx.compose.foundation.border
import androidx.compose.foundation.clickable
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.heightIn
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.lazy.LazyColumn
import androidx.compose.foundation.lazy.items
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.text.TextStyle
import androidx.compose.ui.text.font.FontFamily
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.tooling.preview.Preview
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import com.ntu.mdp.t7.feature.task1_arena.ArenaColors
import com.ntu.mdp.t7.ui.theme.MyApplicationTheme

@Composable
fun ProtocolLogDrawer(messages: List<String>, modifier: Modifier = Modifier) {
    var expanded by remember { mutableStateOf(false) }
    ProtocolLogDrawerContent(
        messages = messages,
        expanded = expanded,
        onToggle = { expanded = !expanded },
        modifier = modifier,
    )
}

@Composable
fun ProtocolLogDrawerContent(
    messages: List<String>,
    expanded: Boolean,
    onToggle: () -> Unit,
    modifier: Modifier = Modifier,
) {
    Column(
        modifier = modifier
            .background(ArenaColors.SurfaceWhite, RoundedCornerShape(12.dp))
            .border(1.dp, ArenaColors.Border, RoundedCornerShape(12.dp))
            .padding(12.dp),
        verticalArrangement = Arrangement.spacedBy(8.dp),
    ) {
        Row(
            modifier = Modifier.fillMaxWidth().clickable(onClick = onToggle),
            horizontalArrangement = Arrangement.SpaceBetween,
            verticalAlignment = Alignment.CenterVertically,
        ) {
            Text(
                text = "Developer: raw protocol log (${messages.size})",
                style = TextStyle(
                    fontSize = 13.sp,
                    fontWeight = FontWeight.Medium,
                    fontFamily = FontFamily.Monospace,
                    color = ArenaColors.TextSecondary,
                ),
            )
            Text(
                text = if (expanded) "▲" else "▼",
                style = TextStyle(fontSize = 13.sp, color = ArenaColors.TextSecondary),
            )
        }

        if (expanded) {
            if (messages.isEmpty()) {
                Text(
                    text = "No traffic yet.",
                    style = TextStyle(fontSize = 13.sp, fontFamily = FontFamily.Monospace, color = ArenaColors.TextSecondary),
                )
            } else {
                LazyColumn(
                    modifier = Modifier.heightIn(max = 160.dp),
                    verticalArrangement = Arrangement.spacedBy(4.dp),
                ) {
                    items(messages) { message ->
                        Text(
                            text = message,
                            style = TextStyle(fontSize = 13.sp, fontFamily = FontFamily.Monospace, color = ArenaColors.TextPrimary),
                        )
                    }
                }
            }
        }
    }
}
