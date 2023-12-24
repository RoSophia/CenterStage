package org.firstinspires.ftc.teamcode.utils

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.hardware.Swerve
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.endma
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.initma
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.log
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.preinit
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.send_log
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.shutUp
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.swerve
import org.firstinspires.ftc.teamcode.utils.RobotVars.OFFLB
import org.firstinspires.ftc.teamcode.utils.RobotVars.OFFLF
import org.firstinspires.ftc.teamcode.utils.RobotVars.OFFRB
import org.firstinspires.ftc.teamcode.utils.RobotVars.OFFRF
import org.firstinspires.ftc.teamcode.utils.RobotVars.nrRots

@TeleOp(name = "設置變量")
class SetVars : OpMode() {
    override fun init() {
        if (nrRots == null) {
            nrRots = HashMap()
        }
        OFFLF = 0.0
        OFFLB = 0.0
        OFFRF = 0.0
        OFFRB = 0.0
        shutUp(this)
        OFFLF = -swerve.lf.s.e.pos
        OFFLB = -swerve.lb.s.e.pos
        OFFRF = -swerve.rf.s.e.pos
        OFFRB = -swerve.rb.s.e.pos
        log("public static double OFFLF = $OFFLF;", "")
        log("public static double OFFLB = $OFFLB;", "")
        log("public static double OFFRF = $OFFRF;", "")
        log("public static double OFFRB = $OFFRB;", "")
        nrRots.clear()
        send_log()
    }

    override fun loop() {}

    override fun stop() {
        endma()
    }
}