package org.firstinspires.ftc.teamcode.hardware

import com.acmerobotics.dashboard.FtcDashboard
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl
import org.firstinspires.ftc.teamcode.auto.AutoFuncs
import org.firstinspires.ftc.teamcode.auto.AutoFuncs.initQrCamera
import org.firstinspires.ftc.teamcode.utils.RobotFuncs
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.create_god
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.hardwareMap
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.log
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.logst
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.lom
import org.firstinspires.ftc.teamcode.utils.RobotVars.CameraExposure
import org.firstinspires.ftc.teamcode.utils.RobotVars.CameraGain
import org.firstinspires.ftc.teamcode.utils.RobotVars.QrName
import org.firstinspires.ftc.teamcode.utils.RobotVars.USE_CAMERA_DETECTION
import org.firstinspires.ftc.teamcode.utils.TrajectorySequence
import org.openftc.easyopencv.*
import java.lang.Thread.sleep
import java.util.concurrent.TimeUnit

class CamGirl(
        val name: String,
        val orientation: OpenCvCameraRotation,
        val resX: Int,
        val resY: Int,
        pipeline: OpenCvPipeline?,
        val streaming: Boolean,
        waitForOpen: Boolean,
        val gain: Int,
        val exposure: Int) {

    constructor(
            name: String,
            orientation: OpenCvCameraRotation,
            resX: Int,
            resY: Int,
            pipeline: OpenCvPipeline?
    ) : this(name, orientation, resX, resY, pipeline, false, false)

    constructor(
            name: String,
            orientation: OpenCvCameraRotation,
            resX: Int,
            resY: Int,
            pipeline: OpenCvPipeline?,
            streaming: Boolean,
            waitForOpen: Boolean,
    ) : this(name, orientation, resX, resY, pipeline, streaming, waitForOpen, CameraGain, CameraExposure)

    var camera: OpenCvWebcam
    var ecode: Int = 0
    var opened: Boolean = false
    private var dashboardStreaming = false

    init {
        val cameraMonitorViewId: Int = hardwareMap.appContext.resources.getIdentifier("cameraMonitorViewId", "id", lom.hardwareMap.appContext.packageName)
        val webcamName: WebcamName = hardwareMap.get(WebcamName::class.java, name)
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId)
        if (pipeline != null) {
            camera.setPipeline(pipeline)
        }

        dashboardStreaming = streaming
        val cameraListener = object : OpenCvCamera.AsyncCameraOpenListener {
            override fun onOpened() {
                if (exposure != 0) {
                    camera.exposureControl.mode = ExposureControl.Mode.Manual
                    sleep(50)
                    camera.exposureControl.setExposure(exposure.toLong(), TimeUnit.MILLISECONDS)
                }
                camera.gainControl.gain = gain
                /// TODO I LOVEWHAITBALES

                camera.startStreaming(resX, resY, orientation,
                        if (name == QrName) OpenCvWebcam.StreamFormat.MJPEG else OpenCvWebcam.StreamFormat.YUY2)
                if (streaming) {
                    FtcDashboard.getInstance().startCameraStream(camera, 120.0)
                }
                opened = true
            }

            override fun onError(errorCode: Int) {
                ecode = errorCode
            }
        }

        camera.openCameraDeviceAsync(cameraListener)
        while (waitForOpen && !opened && !lom.isStopRequested && !lom.isStarted) {
            lom.telemetry.addLine("Waiting on cam open")
            lom.telemetry.update()
            sleep(5)
        }
        logst("Cam opened")
        lom.telemetry.addLine("Cam opened")
        lom.telemetry.update()
    }

    fun stop() {
        if (opened) {
            camera.closeCameraDeviceAsync {
                opened = false
                if (USE_CAMERA_DETECTION && !lom.isStopRequested) {
                    sleep(150)
                    initQrCamera()
                }
            }
            /*
            if (dashboardStreaming) {
                FtcDashboard.getInstance().stopCameraStream()
                TrajectorySequence().sl(0.5).aa { create_god() } .runAsync()
            }
             */
        }
    }
}