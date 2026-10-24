package com.ntu.mdp.t7.ui.theme

import androidx.compose.ui.text.TextStyle
import androidx.compose.ui.text.font.FontFamily
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.sp

val DataFontFamily: FontFamily = FontFamily.Monospace

object AppType {
    val Display = TextStyle(fontFamily = DataFontFamily, fontSize = 30.sp, fontWeight = FontWeight.SemiBold)
    val Title = TextStyle(fontFamily = FontFamily.Default, fontSize = 22.sp, fontWeight = FontWeight.SemiBold)
    val Subtitle = TextStyle(fontFamily = FontFamily.Default, fontSize = 17.sp, fontWeight = FontWeight.Medium)
    val Body = TextStyle(fontFamily = FontFamily.Default, fontSize = 15.sp, fontWeight = FontWeight.Normal)
    val Label = TextStyle(fontFamily = FontFamily.Default, fontSize = 13.sp, fontWeight = FontWeight.Medium)
    val Mono = TextStyle(fontFamily = DataFontFamily, fontSize = 13.sp, fontWeight = FontWeight.Normal)
    val Caption = TextStyle(fontFamily = DataFontFamily, fontSize = 11.sp, fontWeight = FontWeight.Medium)
}
