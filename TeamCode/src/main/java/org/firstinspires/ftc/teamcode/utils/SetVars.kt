package org.firstinspires.ftc.teamcode.utils

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.outoftheboxrobotics.photoncore.Photon
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.dashboard
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.endma
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.shutUp
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.swerve
import org.firstinspires.ftc.teamcode.utils.RobotVars.SwerveWheelOffsets

@Photon
@TeleOp(name = "設置變量")
class SetVars : LinearOpMode() {
    override fun runOpMode() {
        SwerveWheelOffsets.x = 0.0
        SwerveWheelOffsets.y = 0.0
        SwerveWheelOffsets.z = 0.0
        SwerveWheelOffsets.w = 0.0
        shutUp(this)
        val OFFLF = -swerve.lf.s.e.angle
        val OFFLB = -swerve.lb.s.e.angle
        val OFFRF = -swerve.rf.s.e.angle
        val OFFRB = -swerve.rb.s.e.angle
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
