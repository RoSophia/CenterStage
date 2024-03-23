package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.FtcDashboard
import com.outoftheboxrobotics.photoncore.Photon
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.util.ElapsedTime
import com.qualcomm.robotcore.util.Range
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl
import org.firstinspires.ftc.teamcode.auto.AutoFuncs.initQrCamera
import org.firstinspires.ftc.teamcode.hardware.Timmy
import org.firstinspires.ftc.teamcode.pp.ThreeWheelLocalizer
import org.firstinspires.ftc.teamcode.pp.TrajCoef
import org.firstinspires.ftc.teamcode.pp.Trajectory
import org.firstinspires.ftc.teamcode.utils.Pose
import org.firstinspires.ftc.teamcode.utils.RobotFuncs
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.KILLALL
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.aprilTag
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.create_god
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.endma
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.fptp
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.initma
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.log
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.logst
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.moveSwerve
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.pp
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.preinit
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.send_log
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.update
import org.firstinspires.ftc.teamcode.utils.RobotVars
import org.firstinspires.ftc.teamcode.utils.RobotVars.KILLMYSELF
import org.firstinspires.ftc.teamcode.utils.RobotVars.WheelsCamAng
import org.firstinspires.ftc.teamcode.utils.TrajectorySequence
import org.firstinspires.ftc.teamcode.utils.Vec2d
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor
import java.util.concurrent.TimeUnit
import kotlin.math.PI

@Photon
@TeleOp
class CreateGod : LinearOpMode() {

    private fun ts(p: AprilTagPoseFtc) = String.format("xyz: %.2f %.2f %.2f\nyrp: %.2f %.2f %.2f\nrang: %.2f, baer: %.2f, elev: %.2f", p.x, p.y, p.z, p.yaw, p.roll, p.pitch, p.range, p.bearing, p.elevation)
    override fun runOpMode() {
        preinit()
        initma(this, false)

        val t = TrajectorySequence()
                .st(2)
                .aa { logst("KMKSM1") }
                .sl(0.1)
                .gt { if (gamepad1.a) 3 else 2 }
                .st(3)
                .at(Trajectory(TrajCoef(Pose(), Pose(100.0, 0.0, 0.0))))
                .gt { 3 }

        waitForStart()

        t.reset()
        while (!isStopRequested && !t.update()) {
            pp.update()
            update()
        }


        create_god()
        /*

        initQrCamera()

        while (!isStopRequested) {
            val det = aprilTag?.detections
            RobotFuncs.localizer.update()
            if (det != null) {
                for (detection in det) {
                    logst("Detected id ${detection.id} at ${fptp(detection.ftcPose)} : ${detection.decisionMargin} : ${detection.frameAcquisitionNanoTime}")
                }
            }
            moveSwerve()
            update()
            send_log()
        }
         */

        endma()
    }

}
