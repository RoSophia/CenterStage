package org.firstinspires.ftc.teamcode.hardware

import android.R.attr
import org.firstinspires.ftc.teamcode.utils.Util.angDiff
import org.firstinspires.ftc.teamcode.utils.Util.epsEq
import kotlin.math.PI


class Swerve {
    val lf = SwerveModule("LF")
    val lb = SwerveModule("LB")
    val rf = SwerveModule("RF")
    val rb = SwerveModule("RB")

    fun turn(turnPower: Double) {
        lf.angle = PI * 3 / 4
        lb.angle = PI * 1 / 4
        rf.angle = PI * 7 / 4
        rb.angle = PI * 5 / 4

        lf.speed = turnPower
        lb.speed = turnPower
        rf.speed = turnPower
        rb.speed = turnPower
    }

    var speed = 0.0
    var angle = 0.0
    var turnPower = 0.0

    fun move(speed: Double, angle: Double, turnPower: Double) {
        if (epsEq(this.speed, speed) && epsEq(this.angle, angle) && epsEq(this.turnPower, turnPower)) {
            return
        }

        this.speed = speed
        this.angle = angle
        this.turnPower = turnPower

        if (epsEq(speed, 0.0) && !epsEq(turnPower, 0.0)) {
            turn(turnPower)
        } else {
            val turnAngle = turnPower * 45.0

            lf.angle = angle + if (angDiff(angle, PI * 3 / 4) >= PI / 2) turnAngle else -turnAngle
            lb.angle = angle + if (angDiff(angle, PI * 5 / 4) > PI / 2) turnAngle else -turnAngle
            rf.angle = angle + if (angDiff(angle, PI * 1 / 4) > PI / 2) turnAngle else -turnAngle
            rb.angle = angle + if (angDiff(angle, PI * 7 / 4) >= PI / 2) turnAngle else -turnAngle

            lf.speed = speed
            lb.speed = speed
            rf.speed = speed
            rb.speed = speed
        }
    }

    fun close() {
        lf.close()
        lb.close()
        rf.close()
        rb.close()
    }

}