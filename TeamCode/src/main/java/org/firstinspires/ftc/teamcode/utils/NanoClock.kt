package org.firstinspires.ftc.teamcode.utils

/**
 * Clock interface with nanosecond precision and no guarantee about its origin (that is, this is only suited for
 * measuring relative/elapsed time).
 */
// interface breaks companion object JVM static modifier
class NanoClock {

    var lt = System.nanoTime()

    fun seconds(): Double {
        val ct = System.nanoTime()
        val dt = (ct - lt) / 1e9
        lt = ct
        return dt
    }

    fun reset() {
        lt = System.nanoTime()
    }
}
