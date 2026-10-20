package com.example.myapplication.arena.components

import androidx.compose.foundation.background
import androidx.compose.foundation.border
import androidx.compose.foundation.clickable
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import com.example.myapplication.ui.theme.AppType
import com.example.myapplication.ui.theme.Dimens

/** Shared pill-shaped text button used by LinkBar and DevicePickerSheet. */
@Composable
fun PillButton(label: String, onClick: () -> Unit, modifier: Modifier = Modifier) {
    Box(
        modifier = modifier
            .height(Dimens.TouchTargetMin)
            .background(MaterialTheme.colorScheme.surface, RoundedCornerShape(Dimens.RadiusPill))
            .border(Dimens.BorderWidth, MaterialTheme.colorScheme.outline, RoundedCornerShape(Dimens.RadiusPill))
            .clickable(onClick = onClick)
            .padding(horizontal = Dimens.Space16),
        contentAlignment = Alignment.Center,
    ) {
        Text(text = label, style = AppType.Label, color = MaterialTheme.colorScheme.onSurface)
    }
}
