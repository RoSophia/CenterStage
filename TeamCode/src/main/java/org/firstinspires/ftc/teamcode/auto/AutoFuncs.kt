package org.firstinspires.ftc.teamcode.auto

import android.util.Size
import com.acmerobotics.dashboard.FtcDashboard
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.util.Range
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl
import org.firstinspires.ftc.teamcode.hardware.CamGirl
import org.firstinspires.ftc.teamcode.hardware.CameraControls.AutoRed
import org.firstinspires.ftc.teamcode.hardware.CameraControls.AutoResult
import org.firstinspires.ftc.teamcode.hardware.CameraControls.QrExposure
import org.firstinspires.ftc.teamcode.hardware.CameraControls.QrGain
import org.firstinspires.ftc.teamcode.pp.PP.JustDraw
import org.firstinspires.ftc.teamcode.utils.RobotFuncs
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.aprilTag
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.cam
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.dashboard
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.endma
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.hardwareMap
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.initAuto
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.initma
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.log
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.lom
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.pp
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.preinit
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.send_log
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.startma
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.update
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.visionPortal
import org.firstinspires.ftc.teamcode.utils.RobotVars
import org.firstinspires.ftc.teamcode.utils.RobotVars.CameraName
import org.firstinspires.ftc.teamcode.utils.RobotVars.CameraOrientation
import org.firstinspires.ftc.teamcode.utils.RobotVars.CameraX
import org.firstinspires.ftc.teamcode.utils.RobotVars.CameraY
import org.firstinspires.ftc.teamcode.utils.RobotVars.TimmyCurOff
import org.firstinspires.ftc.teamcode.utils.RobotVars.TimmyToClose
import org.firstinspires.ftc.teamcode.utils.RobotVars.USE_CAMERA
import org.firstinspires.ftc.teamcode.utils.RobotVars.USE_CAMERA_DETECTION
import org.firstinspires.ftc.teamcode.utils.RobotVars.__AutoShort
import org.firstinspires.ftc.teamcode.utils.RobotVars.__LOG_STATUS
import org.firstinspires.ftc.teamcode.utils.RobotVars.timmy
import org.firstinspires.ftc.teamcode.utils.TrajectorySequence
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor
import java.lang.Thread.sleep
import java.util.concurrent.TimeUnit
import kotlin.math.PI

object AutoFuncs {

    private fun updateAuto(e: TrajectorySequence, func: () -> TrajectorySequence): TrajectorySequence? {
        if (JustDraw) {
            val ce = func()
            ce.draw()
            update()
            return ce
        } else {
            if (e.update()) {
                lom.requestOpModeStop()
            }
            pp.update()
        }
        update()
        return null
    }

    fun setupAuto(lom: LinearOpMode, isRed: Boolean, isShort: Boolean, func: () -> TrajectorySequence) {
        totBordu(lom, isRed, isShort)
        var e = func()

        while (!lom.isStopRequested) {
            val ce = updateAuto(e, func)
            if (ce != null) {
                e = ce
            }
        }

        TimmyToClose = true
        log("OldTimmyCurOff", TimmyCurOff)
        TimmyCurOff = 0.0
        //TimmyCurOff = timmy.yaw
        log("NewTimmyCurOff", TimmyCurOff)
        send_log()
        endma()
    }

    fun totBordu(lom: LinearOpMode, isRed: Boolean, isShort: Boolean): Int {
        TimmyToClose = true
        AutoRed = isRed
        __AutoShort = isShort
        preinit()
        initma(lom, true)
        initAuto()
        lom.waitForStart()
        val res = AutoResult // Pun asta aici sa nu faca amuzant inchiderea camerei
        if (USE_CAMERA) {
            cam?.stop()
        }
        startma()
        TimmyCurOff = 0.0
        return res
    }

    fun updateAutoCustom(e: TrajectorySequence, func: () -> TrajectorySequence): TrajectorySequence? {
        if (JustDraw) {
            val ce = func(); ce.draw(); update(); return ce
        } else {
            if (e.update()) {
                return func()
            }
            pp.update()
        }
        update()
        return null
    }

    fun initQrCamera() {
        if (USE_CAMERA_DETECTION) {
            if (cam != null) {
                cam?.stop()
                //cam = CamGirl(CameraName, CameraOrientation, 640, 480, null, streaming = true, waitForOpen = true)
            }
            val ccam = hardwareMap.get(WebcamName::class.java, "Anticamera")
            aprilTag = AprilTagProcessor.Builder()
                    .setLensIntrinsics(822.317, 822.317, 319.495, 242.502)
                    .build()
            visionPortal = VisionPortal.Builder()
                    .setCamera(ccam)
                    .setCameraResolution(Size(CameraX, CameraY))
                    .setShowStatsOverlay(false)
                    .addProcessor(aprilTag)
                    .build()

            dashboard.startCameraStream(visionPortal, 10.0)

            while (!lom.isStopRequested && visionPortal.cameraState != VisionPortal.CameraState.STREAMING) {
                sleep(10)
            }
            val exposureControl = visionPortal.getCameraControl(ExposureControl::class.java)
            if (exposureControl.mode != ExposureControl.Mode.Manual) {
                exposureControl.mode = ExposureControl.Mode.Manual
                sleep(50)
            }
            exposureControl.setExposure(QrExposure.toLong(), TimeUnit.MILLISECONDS)
            sleep(20)
            visionPortal.getCameraControl(GainControl::class.java).gain = QrGain
        }
    }
}