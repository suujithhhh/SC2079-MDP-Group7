package com.ntu.mdp.t7.core.protocol

import org.junit.Assert.assertEquals
import org.junit.Test

class LineFramerTest {

    @Test
    fun `returns nothing until a terminator arrives`() {
        val framer = LineFramer()
        assertEquals(emptyList<String>(), framer.feed("ROBOT,7,2"))
    }

    @Test
    fun `a message split across two reads is reassembled`() {
        val framer = LineFramer()
        assertEquals(emptyList<String>(), framer.feed("ROBOT,7,"))
        assertEquals(listOf("ROBOT,7,2,N"), framer.feed("2,N\n"))
    }

    @Test
    fun `two messages concatenated in one read are both extracted in order`() {
        val framer = LineFramer()
        val lines = framer.feed("ROBOT,7,2,N\nMSG,[Moving]\n")
        assertEquals(listOf("ROBOT,7,2,N", "MSG,[Moving]"), lines)
    }

    @Test
    fun `tolerates a CRLF terminator`() {
        val framer = LineFramer()
        assertEquals(listOf("ROBOT,7,2,N"), framer.feed("ROBOT,7,2,N\r\n"))
    }

    @Test
    fun `tolerates CR carriage return terminator`() {
        val framer = LineFramer()
        val lines = framer.feed("TARGET,B1,20,N\rTARGET,B2,21,E\r")
        assertEquals(listOf("TARGET,B1,20,N", "TARGET,B2,21,E"), lines)
    }

    @Test
    fun `partial data after the last terminator is retained for the next feed`() {
        val framer = LineFramer()
        val first = framer.feed("MSG,[Moving]\nROBOT,7,")
        assertEquals(listOf("MSG,[Moving]"), first)
        val second = framer.feed("2,N\n")
        assertEquals(listOf("ROBOT,7,2,N"), second)
    }

    @Test
    fun `reset discards a partial unterminated line`() {
        val framer = LineFramer()
        framer.feed("ROBOT,7,")
        framer.reset()
        assertEquals(listOf("MSG,[Moving]"), framer.feed("MSG,[Moving]\n"))
    }

    @Test
    fun `buffer does not grow unbounded when no terminator ever arrives`() {
        val framer = LineFramer(maxBufferLength = 16)
        framer.feed("a".repeat(20))
        assertEquals(listOf("ROBOT,7,2,N"), framer.feed("ROBOT,7,2,N\n"))
    }
}
