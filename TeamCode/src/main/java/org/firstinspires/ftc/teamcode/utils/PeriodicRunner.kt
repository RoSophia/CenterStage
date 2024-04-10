package org.firstinspires.ftc.teamcode.utils

import org.firstinspires.ftc.teamcode.utils.RobotFuncs.etime
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.log
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.logs
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.lom

class PeriodicRunner<T>(shouldRead: () -> Boolean, func: () -> T, writer: (T) -> Unit, defaultValue: T, refreshRate: Double, name: String) {
    constructor(shouldRead: () -> Boolean, func: () -> T, writer: (T) -> Unit, defaultValue: T) : this(shouldRead, func, writer, defaultValue, 10.0, "NoNamePeriodicRunner")

    private val thread = Thread {
        try {
            writer(defaultValue)
            while (!lom.isStopRequested) {
                if (lom.opModeIsActive() && shouldRead()) {
                    val r = func()
                    log("G$name", "${r.toString()} at ${etime.seconds()}")
                    writer(r)
                } else {
                    writer(defaultValue)
                }
                Thread.sleep((1000.0 / refreshRate).toLong())
            }
        } catch (e: Exception) {
            log("Periodicrunner exception $name", e.stackTraceToString())
        }
    }

    init {
        thread.start()
    }

    fun stop() {
        thread.join()
    }
}