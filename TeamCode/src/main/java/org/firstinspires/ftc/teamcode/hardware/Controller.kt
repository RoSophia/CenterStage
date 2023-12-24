package org.firstinspires.ftc.teamcode.hardware

import org.firstinspires.ftc.teamcode.utils.RobotFuncs

@Suppress("PrivatePropertyName", "MemberVisibilityCanBePrivate", "PropertyName")
class Controller {
    private val g1 = RobotFuncs.lom.gamepad1
    private val g2 = RobotFuncs.lom.gamepad2

    private val TRIGGER_THRESHOLD = 0.6

    val JUST_PRESSED = 2
    val JUST_RELEASED = 1
    val RELEASED = 0
    val PRESSED = 3

    private var L1A = g1.a
    private var L1B = g1.b
    private var L1X = g1.x
    private var L1Y = g1.y
    private var L1DD = g1.dpad_down
    private var L1DU = g1.dpad_up
    private var L1DL = g1.dpad_left
    private var L1DR = g1.dpad_right
    private var L1RSB = g1.right_stick_button
    private var L1RB = g1.right_bumper
    private var L1RT = g1.right_trigger > TRIGGER_THRESHOLD
    private var L1LSB = g1.left_stick_button
    private var L1LB = g1.left_bumper
    private var L1LT = g1.left_trigger > TRIGGER_THRESHOLD
    private var L1PS = g1.ps
    private var L1START = g1.start
    private var L1OPT = g1.options

    var C1A = 0
    var C1B = 0
    var C1X = 0
    var C1Y = 0
    var C1DD = 0
    var C1DU = 0
    var C1DL = 0
    var C1DR = 0
    var C1RSB = 0
    var C1RB = 0
    var C1RT = 0
    var C1LSB = 0
    var C1LB = 0
    var C1LT = 0
    var C1PS = 0
    var C1START = 0
    var C1OPT = 0

    private var L2A = g2.a
    private var L2B = g2.b
    private var L2X = g2.x
    private var L2Y = g2.y
    private var L2DD = g2.dpad_down
    private var L2DU = g2.dpad_up
    private var L2DL = g2.dpad_left
    private var L2DR = g2.dpad_right
    private var L2RSB = g2.right_stick_button
    private var L2RB = g2.right_bumper
    private var L2RT = g2.right_trigger > TRIGGER_THRESHOLD
    private var L2LSB = g2.left_stick_button
    private var L2LB = g2.left_bumper
    private var L2LT = g2.left_trigger > TRIGGER_THRESHOLD
    private var L2PS = g2.ps
    private var L2START = g2.start
    private var L2OPT = g2.options

    var C2A = 0
    var C2B = 0
    var C2X = 0
    var C2Y = 0
    var C2DD = 0
    var C2DU = 0
    var C2DL = 0
    var C2DR = 0
    var C2RSB = 0
    var C2RB = 0
    var C2RT = 0
    var C2LSB = 0
    var C2LB = 0
    var C2LT = 0
    var C2PS = 0
    var C2START = 0
    var C2OPT = 0

    fun toInt(o: Boolean): Int {
        return if (o) return 1 else 0
    }

    fun update() {
        C1A = 2 * toInt(g1.a) + toInt(L1A)
        C1B = 2 * toInt(g1.b) + toInt(L1B)
        C1X = 2 * toInt(g1.x) + toInt(L1X)
        C1Y = 2 * toInt(g1.y) + toInt(L1Y)
        C1DD = 2 * toInt(g1.dpad_down) + toInt(L1DD)
        C1DU = 2 * toInt(g1.dpad_up) + toInt(L1DU)
        C1DL = 2 * toInt(g1.dpad_left) + toInt(L1DL)
        C1DR = 2 * toInt(g1.dpad_right) + toInt(L1DR)
        C1RSB = 2 * toInt(g1.right_stick_button) + toInt(L1RSB)
        C1RB = 2 * toInt(g1.right_bumper) + toInt(L1RB)
        C1RT = 2 * toInt(g1.right_trigger > TRIGGER_THRESHOLD) + toInt(L1RT)
        C1LSB = 2 * toInt(g1.left_stick_button) + toInt(L1LSB)
        C1LB = 2 * toInt(g1.left_bumper) + toInt(L1LB)
        C1LT = 2 * toInt(g1.left_trigger > TRIGGER_THRESHOLD) + toInt(L1LT)
        C1PS = 2 * toInt(g1.ps) + toInt(L1PS)
        C1START = 2 * toInt(g1.start) + toInt(L1START)
        C1OPT = 2 * toInt(g1.options) + toInt(L1OPT)

        L1A = g1.a
        L1B = g1.b
        L1X = g1.x
        L1Y = g1.y
        L1DD = g1.dpad_down
        L1DU = g1.dpad_up
        L1DL = g1.dpad_left
        L1DR = g1.dpad_right
        L1RSB = g1.right_stick_button
        L1RB = g1.right_bumper
        L1RT = g1.right_trigger > TRIGGER_THRESHOLD
        L1LSB = g1.left_stick_button
        L1LB = g1.left_bumper
        L1LT = g1.left_trigger > TRIGGER_THRESHOLD
        L1PS = g1.ps
        L1START = g1.start
        L1OPT = g1.options

        C2A = 2 * toInt(g2.a) + toInt(L2A)
        C2B = 2 * toInt(g2.b) + toInt(L2B)
        C2X = 2 * toInt(g2.x) + toInt(L2X)
        C2Y = 2 * toInt(g2.y) + toInt(L2Y)
        C2DD = 2 * toInt(g2.dpad_down) + toInt(L2DD)
        C2DU = 2 * toInt(g2.dpad_up) + toInt(L2DU)
        C2DL = 2 * toInt(g2.dpad_left) + toInt(L2DL)
        C2DR = 2 * toInt(g2.dpad_right) + toInt(L2DR)
        C2RSB = 2 * toInt(g2.right_stick_button) + toInt(L2RSB)
        C2RB = 2 * toInt(g2.right_bumper) + toInt(L2RB)
        C2RT = 2 * toInt(g2.right_trigger > TRIGGER_THRESHOLD) + toInt(L2RT)
        C2LSB = 2 * toInt(g2.left_stick_button) + toInt(L2LSB)
        C2LB = 2 * toInt(g2.left_bumper) + toInt(L2LB)
        C2LT = 2 * toInt(g2.left_trigger > TRIGGER_THRESHOLD) + toInt(L2LT)
        C2PS = 2 * toInt(g2.ps) + toInt(L2PS)
        C2START = 2 * toInt(g2.start) + toInt(L2START)
        C2OPT = 2 * toInt(g2.options) + toInt(L2OPT)

        L2A = g2.a
        L2B = g2.b
        L2X = g2.x
        L2Y = g2.y
        L2DD = g2.dpad_down
        L2DU = g2.dpad_up
        L2DL = g2.dpad_left
        L2DR = g2.dpad_right
        L2RSB = g2.right_stick_button
        L2RB = g2.right_bumper
        L2RT = g2.right_trigger > TRIGGER_THRESHOLD
        L2LSB = g2.left_stick_button
        L2LB = g2.left_bumper
        L2LT = g2.left_trigger > TRIGGER_THRESHOLD
        L2PS = g2.ps
        L2START = g2.start
        L2OPT = g2.options
    }
}
