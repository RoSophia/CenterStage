package org.firstinspires.ftc.teamcode.hardware

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.dashboard
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.log
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.logs
import org.firstinspires.ftc.teamcode.utils.RobotVars.OFFLB
import org.firstinspires.ftc.teamcode.utils.RobotVars.OFFLF
import org.firstinspires.ftc.teamcode.utils.RobotVars.OFFRB
import org.firstinspires.ftc.teamcode.utils.RobotVars.OFFRF
import org.firstinspires.ftc.teamcode.utils.Util.angDiff
import org.firstinspires.ftc.teamcode.utils.Util.angNorm
import org.firstinspires.ftc.teamcode.utils.Util.epsEq
import kotlin.math.PI

class Swerve {
    val lf = SwerveModule("LF", OFFLF)
    val lb = SwerveModule("LB", OFFLB)
    val rf = SwerveModule("RF", OFFRF)
    val rb = SwerveModule("RB", OFFRB)

    init {
        logs("Swerve_Status", "Init");
    }

    fun turn(turnPower: Double) {
        lf.angle = angNorm(PI * (1 + 4) / 4)
        lb.angle = angNorm(PI * (3 + 0) / 4)
        rf.angle = angNorm(PI * (7 + 0) / 4)
        rb.angle = angNorm(PI * (5 + 4) / 4)

        lf.speed = turnPower
        lb.speed = turnPower
        rf.speed = turnPower
        rb.speed = turnPower
    }

    var speed = 0.0
    var angle = 0.0
    var turnPower = 0.0

    fun move(speed: Double, angle: Double, turnPower: Double) {
        log("Swerve_Movement", "${speed}@${angle} tp: $turnPower")
        if (epsEq(this.speed, speed) && epsEq(this.angle, angle) && epsEq(this.turnPower, turnPower)) {
            return
        }

        this.turnPower = turnPower
        this.speed = speed
        lf.speed = speed
        lb.speed = speed
        rf.speed = speed
        rb.speed = speed

        if (!epsEq(speed, 0.0)) {
            val turnAngle = turnPower * 45.0

            this.angle = angle
            lf.angle = angle + if (angDiff(angle, PI * 3 / 4) >= PI / 2) turnAngle else -turnAngle
            lb.angle = angle + if (angDiff(angle, PI * 5 / 4) > PI / 2) turnAngle else -turnAngle
            rf.angle = angle + if (angDiff(angle, PI * 1 / 4) > PI / 2) turnAngle else -turnAngle
            rb.angle = angle + if (angDiff(angle, PI * 7 / 4) >= PI / 2) turnAngle else -turnAngle
        } else if (!epsEq(turnPower, 0.0)) {
            turn(turnPower)
        }
    }

    fun close() {
        logs("Swerve_Status", "InitClose");
        lf.close()
        lb.close()
        rf.close()
        rb.close()

        logs("Swerve_Status", "FinishClose");
    }

}
