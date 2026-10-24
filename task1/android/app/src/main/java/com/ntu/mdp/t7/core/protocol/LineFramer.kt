package com.ntu.mdp.t7.core.protocol

/**
 * Buffers a Bluetooth Classic byte/character stream into discrete lines.
 */
class LineFramer(private val maxBufferLength: Int = 4096) {
    private val buffer = StringBuilder()

    fun feed(chunk: String): List<String> {
        buffer.append(chunk)
        val lines = mutableListOf<String>()
        var i = 0
        var lineStart = 0
        while (i < buffer.length) {
            val ch = buffer[i]
            if (ch == '\n' || ch == '\r') {
                val rawLine = buffer.substring(lineStart, i).trim()
                if (rawLine.isNotEmpty()) {
                    lines.add(rawLine)
                }
                if (ch == '\r' && i + 1 < buffer.length && buffer[i + 1] == '\n') {
                    i++
                }
                lineStart = i + 1
            }
            i++
        }
        if (lineStart > 0) {
            buffer.delete(0, lineStart)
        }

        if (buffer.length > maxBufferLength) {
            buffer.clear()
        }
        return lines
    }

    fun reset() {
        buffer.clear()
    }

    fun flushPending(): String? {
        if (buffer.isEmpty()) return null
        val pending = buffer.toString().trim()
        buffer.clear()
        return pending.ifEmpty { null }
    }
}
