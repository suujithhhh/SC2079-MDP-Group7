package com.example.myapplication.protocol

/**
 * Buffers a Bluetooth Classic byte/character stream into discrete lines. RFCOMM gives a
 * raw byte stream: a single read can contain multiple messages, or a fraction of one.
 * Terminates on `\n`, tolerating a preceding `\r`.
 */
class LineFramer(private val maxBufferLength: Int = 4096) {
    private val buffer = StringBuilder()

    /** Feeds one chunk of received text; returns zero or more complete lines extracted from it. */
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

    /** Discards any partial, unterminated line. Call this when starting a fresh connection. */
    fun reset() {
        buffer.clear()
    }

    /** Flushes and returns any buffered, not-yet-terminated content as a line (trimmed), or
     * null if nothing is buffered. Callers use this when they know no more bytes are
     * immediately available (e.g. `InputStream.available() == 0`) so a peer that never sends
     * a line terminator still gets its message surfaced instead of buffered forever. */
    fun flushPending(): String? {
        if (buffer.isEmpty()) return null
        val pending = buffer.toString().trim()
        buffer.clear()
        return pending.ifEmpty { null }
    }
}
