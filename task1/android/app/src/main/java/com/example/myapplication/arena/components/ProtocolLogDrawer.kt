package com.example.myapplication.arena.components

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
import com.example.myapplication.arena.ArenaColors
import com.example.myapplication.ui.theme.MyApplicationTheme

/** Collapsible raw wire-protocol log — both sent ("→ ") and received ("← ") lines — kept
 * separate from the interpreted status feed per docs/design/design-system-brief.md section 10
 * item 3: never show raw protocol strings as the main feed's primary content. Collapsed by
 * default. */
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

@Preview(name = "ProtocolLogDrawer - collapsed", widthDp = 360)
@Composable
private fun ProtocolLogDrawerCollapsedPreview() {
    MyApplicationTheme {
        ProtocolLogDrawerContent(messages = emptyList(), expanded = false, onToggle = {})
    }
}

@Preview(name = "ProtocolLogDrawer - expanded, empty", widthDp = 360)
@Composable
private fun ProtocolLogDrawerExpandedEmptyPreview() {
    MyApplicationTheme {
        ProtocolLogDrawerContent(messages = emptyList(), expanded = true, onToggle = {})
    }
}

@Preview(name = "ProtocolLogDrawer - expanded, with messages", widthDp = 360)
@Composable
private fun ProtocolLogDrawerExpandedPreview() {
    MyApplicationTheme {
        ProtocolLogDrawerContent(
            messages = listOf(
                "→ " +
                    "{\"type\":\"PLAN_REQUEST\",\"protocol_version\":1,\"mode\":\"B2\",\"coordinate_units\":\"grid\"," +
                    "\"start\":{\"x\":0,\"y\":0,\"heading\":\"N\"},\"obstacles\":[{\"id\":1,\"x\":5,\"y\":12,\"face\":\"N\"}]}",
                "← ROBOT,7,2,N",
                "← TARGET,B1,11,N",
            ),
            expanded = true,
            onToggle = {},
        )
    }
}
