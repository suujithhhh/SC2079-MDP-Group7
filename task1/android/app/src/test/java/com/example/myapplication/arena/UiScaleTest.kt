package com.example.myapplication.arena

import org.junit.Assert.assertEquals
import org.junit.Test

class UiScaleTest {

    @Test
    fun `reference width scales at 1x`() {
        assertEquals(1f, computeUiScale(800f), 0.0001f)
    }

    @Test
    fun `mid-range width scales proportionally`() {
        assertEquals(1.2f, computeUiScale(960f), 0.0001f)
    }

    @Test
    fun `narrow width clamps to the 0,75x floor`() {
        assertEquals(0.75f, computeUiScale(400f), 0.0001f)
    }

    @Test
    fun `wide width clamps to the 1,35x ceiling`() {
        assertEquals(1.35f, computeUiScale(2000f), 0.0001f)
    }
}
