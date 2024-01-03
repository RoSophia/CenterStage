package org.firstinspires.ftc.teamcode.utils

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.hardware.Swerve
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.dashboard
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.diffy
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.endma
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.initma
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.log
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.logst
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.preinit
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.send_log
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.shutUp
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.swerve
import org.firstinspires.ftc.teamcode.utils.RobotVars.CHANGE_DIFFY_POS
import org.firstinspires.ftc.teamcode.utils.RobotVars.DIFLOFF
import org.firstinspires.ftc.teamcode.utils.RobotVars.DIFROFF
import org.firstinspires.ftc.teamcode.utils.RobotVars.OFFLB
import org.firstinspires.ftc.teamcode.utils.RobotVars.OFFLF
import org.firstinspires.ftc.teamcode.utils.RobotVars.OFFRB
import org.firstinspires.ftc.teamcode.utils.RobotVars.OFFRF
import org.firstinspires.ftc.teamcode.utils.RobotVars.nrRots

@TeleOp(name = "設置變量")
class SetVars : LinearOpMode() {
    override fun runOpMode() {
        if (nrRots == null) {
            nrRots = HashMap()
        }
        nrRots.clear()
        OFFLF = 0.0
        OFFLB = 0.0
        OFFRF = 0.0
        OFFRB = 0.0
        if (CHANGE_DIFFY_POS) {
            DIFLOFF = 0.0
            DIFROFF = 0.0
        }
        shutUp(this)
        OFFLF = -swerve.lf.s.e.pos
        OFFLB = -swerve.lb.s.e.pos
        OFFRF = -swerve.rf.s.e.pos
        OFFRB = -swerve.rb.s.e.pos
        if (CHANGE_DIFFY_POS) {
            DIFLOFF = -diffy.LS.e.pos
            DIFROFF = -diffy.RS.e.pos
        }
        val tp = TelemetryPacket()
        tp.addLine("public static double OFFLF = $OFFLF;")
        tp.addLine("public static double OFFLB = $OFFLB;")
        tp.addLine("public static double OFFRF = $OFFRF;")
        tp.addLine("public static double OFFRB = $OFFRB;")
        tp.addLine("public static double DIFLOFF = ${-diffy.LS.e.pos};")
        tp.addLine("public static double DIFROFF = ${-diffy.RS.e.pos};")
        dashboard.sendTelemetryPacket(tp)
        telemetry.addLine("Done!!!")
        telemetry.addLine("Done!!!")
        telemetry.addLine("Done!!!")
        telemetry.addLine("Done!!!")
        telemetry.addLine("Done!!!")
        telemetry.addLine("Done!!!")
        telemetry.addLine("Done!!!")
        telemetry.addLine("Done!!!")
        telemetry.addLine("Done!!!")
        telemetry.update()
        endma()
    }
}