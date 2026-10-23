package com.ntu.mdp.t7.feature.task2_pathing.components

import androidx.compose.foundation.background
import androidx.compose.foundation.border
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.size
import androidx.compose.foundation.shape.CircleShape
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material3.Button
import androidx.compose.material3.ButtonDefaults
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import com.ntu.mdp.t7.ui.theme.Dimens

@Composable
fun Task2ControlPanel(
    onStartClick: () -> Unit,
    isStartSent: Boolean,
    modifier: Modifier = Modifier,
    isReady: Boolean = false,
    isConnected: Boolean = false,
) {
    val enabled = if (isConnected) (isReady && !isStartSent) else !isStartSent

    Column(
        modifier = modifier
            .background(MaterialTheme.colorScheme.surface, RoundedCornerShape(Dimens.RadiusMedium))
            .border(Dimens.BorderWidth, MaterialTheme.colorScheme.outline, RoundedCornerShape(Dimens.RadiusMedium))
            .padding(16.dp),
        horizontalAlignment = Alignment.CenterHorizontally,
        verticalArrangement = Arrangement.spacedBy(16.dp, Alignment.CenterVertically)
    ) {
        Button(
            onClick = onStartClick,
            enabled = enabled,
            colors = ButtonDefaults.buttonColors(
                containerColor = Color.Red,
                disabledContainerColor = Color.Gray.copy(alpha = 0.4f)
            ),
            shape = CircleShape,
            modifier = Modifier.size(120.dp)
        ) {
            Text(
                text = "START",
                color = if (enabled) Color.White else Color.White.copy(alpha = 0.6f),
                fontSize = 20.sp,
                fontWeight = FontWeight.Bold
            )
        }
    }
}
