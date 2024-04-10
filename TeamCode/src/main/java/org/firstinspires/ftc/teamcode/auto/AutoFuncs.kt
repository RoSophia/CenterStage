package org.firstinspires.ftc.teamcode.auto

import android.util.Size
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.WhiteBalanceControl
import org.firstinspires.ftc.teamcode.hardware.CameraControls.AutoRed
import org.firstinspires.ftc.teamcode.hardware.CameraControls.AutoResult
import org.firstinspires.ftc.teamcode.pp.PP.JustDraw
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.aprilTag
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.cam
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.dashboard
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.endma
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.hardwareMap
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.initAuto
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.initma
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.log
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.logst
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.lom
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.pp
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.preinit
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.send_log
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.startma
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.update
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.visionPortal
import org.firstinspires.ftc.teamcode.utils.RobotVars.QrExposure
import org.firstinspires.ftc.teamcode.utils.RobotVars.QrGain
import org.firstinspires.ftc.teamcode.utils.RobotVars.QrThreads
import org.firstinspires.ftc.teamcode.utils.RobotVars.QrX
import org.firstinspires.ftc.teamcode.utils.RobotVars.QrY
import org.firstinspires.ftc.teamcode.utils.RobotVars.TimmyCurOff
import org.firstinspires.ftc.teamcode.utils.RobotVars.TimmyToClose
import org.firstinspires.ftc.teamcode.utils.RobotVars.USE_CAMERA
import org.firstinspires.ftc.teamcode.utils.RobotVars.USE_CAMERA_DETECTION
import org.firstinspires.ftc.teamcode.utils.RobotVars.__AutoShort
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

        TimmyToClose = false
        TimmyCurOff = PI
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
            /*
            if (USE_CAMERA_DETECTION) {
                RobotFuncs.qrpipe = QrDetector(5.0 / 100.0, 450.474, 450.474, 317.178, 186.578)
                RobotFuncs.qrcam = CamGirl(RobotVars.QrName, RobotVars.QrOrientation, RobotVars.QrX, RobotVars.QrY, RobotFuncs.qrpipe,
                        streaming = true, waitForOpen = true, RobotVars.QrGain, RobotVars.QrExposure)
            }
             */
            val ccam = hardwareMap.get(WebcamName::class.java, "Löten")
            aprilTag = AprilTagProcessor.Builder()
                    .setNumThreads(QrThreads)
                    .setLensIntrinsics(680.056, 680.056, 405.776, 307.3)
                    .build()
            visionPortal = VisionPortal.Builder()
                    .setCamera(ccam)
                    .setCameraResolution(Size(QrX, QrY))
                    .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                    .addProcessor(aprilTag)
                    .build()

            dashboard.startCameraStream(visionPortal, 120.0)

            while (!lom.isStopRequested && visionPortal?.cameraState != VisionPortal.CameraState.STREAMING) { sleep(10) }

            val exposureControl = visionPortal?.getCameraControl(ExposureControl::class.java)
            if (exposureControl?.mode != ExposureControl.Mode.Manual) {
                exposureControl?.mode = ExposureControl.Mode.Manual
                while (exposureControl?.mode != ExposureControl.Mode.Manual) {
                    logst("Setting exposure mode")
                    sleep(10)
                }
                logst("Done exposure mode")
            }
            exposureControl.setExposure(QrExposure.toLong(), TimeUnit.MILLISECONDS)
            visionPortal?.getCameraControl(GainControl::class.java)?.gain = QrGain
        }
    }
}