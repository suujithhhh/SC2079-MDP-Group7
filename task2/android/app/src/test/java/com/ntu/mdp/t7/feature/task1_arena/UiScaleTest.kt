package com.ntu.mdp.t7.feature.task1_arena

import org.junit.Assert.assertEquals
import org.junit.Test

class UiScaleTest {

    @Test
    fun `reference width scales at 1x`() {
        assertEquals(1f, computeUiScale(800f, 1200f), 0.0001f)
    }

    @Test
    fun `mid-range width scales proportionally`() {
        assertEquals(1.2f, computeUiScale(960f, 1440f), 0.0001f)
    }

    @Test
    fun `narrow width clamps to the 0,5f floor`() {
        assertEquals(0.5f, computeUiScale(200f, 300f), 0.0001f)
    }

    @Test
    fun `wide width clamps to the 1,35x ceiling`() {
        assertEquals(1.35f, computeUiScale(2000f, 3000f), 0.0001f)
    }
}
