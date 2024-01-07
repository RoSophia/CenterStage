package org.firstinspires.ftc.teamcode.utils

/**
 * Clock interface with nanosecond precision and no guarantee about its origin (that is, this is only suited for
 * measuring relative/elapsed time).
 */
// interface breaks companion object JVM static modifier
class NanoClock {

    var lt = System.nanoTime()

    fun seconds(): Double {
        return (System.nanoTime() - lt) / 1e9
    }

    fun reset() {
        lt = System.nanoTime()
    }
}
