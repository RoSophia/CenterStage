package org.firstinspires.ftc.teamcode.utils

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.outoftheboxrobotics.photoncore.Photon
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.dashboard
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.diffy
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.endma
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.shutUp
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.swerve
import org.firstinspires.ftc.teamcode.utils.RobotVars.OFFLB
import org.firstinspires.ftc.teamcode.utils.RobotVars.OFFLF
import org.firstinspires.ftc.teamcode.utils.RobotVars.OFFRB
import org.firstinspires.ftc.teamcode.utils.RobotVars.OFFRF

@Photon
@TeleOp(name = "設置變量")
class SetVars : LinearOpMode() {
    override fun runOpMode() {
        OFFLF = 0.0
        OFFLB = 0.0
        OFFRF = 0.0
        OFFRB = 0.0
        shutUp(this)
        OFFLF = -swerve.lf.s.e.angle
        OFFLB = -swerve.lb.s.e.angle
        OFFRF = -swerve.rf.s.e.angle
        OFFRB = -swerve.rb.s.e.angle
        val tp = TelemetryPacket()
        tp.addLine("public static double OFFLF = $OFFLF;")
        tp.addLine("public static double OFFLB = $OFFLB;")
        tp.addLine("public static double OFFRF = $OFFRF;")
        tp.addLine("public static double OFFRB = $OFFRB;")
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
