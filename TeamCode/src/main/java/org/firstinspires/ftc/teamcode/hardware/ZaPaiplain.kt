package org.firstinspires.ftc.teamcode.hardware

import com.acmerobotics.dashboard.config.Config
import org.firstinspires.ftc.teamcode.hardware.CameraControls.AutoMinBlocks
import org.firstinspires.ftc.teamcode.hardware.CameraControls.AutoRed
import org.firstinspires.ftc.teamcode.hardware.CameraControls.AutoResult
import org.firstinspires.ftc.teamcode.hardware.CameraControls.COL_INDEX
import org.firstinspires.ftc.teamcode.hardware.CameraControls.CUR_DONE_CORRECTION
import org.firstinspires.ftc.teamcode.hardware.CameraControls.CameraMidPos
import org.firstinspires.ftc.teamcode.hardware.CameraControls.DO_I_EVEN_PROCESS_FRAME
import org.firstinspires.ftc.teamcode.hardware.CameraControls.DRAW_BOXES
import org.firstinspires.ftc.teamcode.hardware.CameraControls.DRAW_MEDIAN
import org.firstinspires.ftc.teamcode.hardware.CameraControls.LUP
import org.firstinspires.ftc.teamcode.hardware.CameraControls.LUT
import org.firstinspires.ftc.teamcode.hardware.CameraControls.PSTEP
import org.firstinspires.ftc.teamcode.hardware.CameraControls.PaiplainColBloo
import org.firstinspires.ftc.teamcode.hardware.CameraControls.PaiplainMaxBloo
import org.firstinspires.ftc.teamcode.hardware.CameraControls.PaiplainMaxRed
import org.firstinspires.ftc.teamcode.hardware.CameraControls.PaiplainMinSat
import org.firstinspires.ftc.teamcode.hardware.CameraControls.PaiplainMinVal
import org.firstinspires.ftc.teamcode.hardware.CameraControls.PaiplainColRed
import org.firstinspires.ftc.teamcode.hardware.CameraControls.XOFF
import org.firstinspires.ftc.teamcode.hardware.CameraControls.YOFF
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.initAuto
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.log
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.lom
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.send_log
import org.firstinspires.ftc.teamcode.utils.RobotVars.__AutoShort
import org.firstinspires.ftc.teamcode.utils.Util.angDiff
import org.opencv.core.Mat
import org.opencv.core.Point
import org.opencv.core.Rect
import org.opencv.core.Scalar
import org.opencv.imgproc.Imgproc
import org.opencv.imgproc.Imgproc.COLOR_RGB2HSV
import org.opencv.imgproc.Imgproc.COLOR_RGB2HSV_FULL
import org.openftc.easyopencv.OpenCvPipeline
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.max

@Config
object CameraControls {
    @JvmField
    var PSTEP: Int = 5

    @JvmField
    var LUP: Int = 100

    @JvmField
    var LUT: Int = 290

    @JvmField
    var XOFF: Int = 320

    @JvmField
    var YOFF: Int = 300

    @JvmField
    var COL_INDEX: Int = 0

    @JvmField
    var DRAW_BOXES: Boolean = true

    @JvmField
    var DRAW_MEDIAN: Boolean = true

    @JvmField
    var DO_I_EVEN_PROCESS_FRAME: Boolean = true

    @JvmField
    var CUR_DONE_CORRECTION: Int = 0

    @JvmField
    var CameraMidPos: Double = 400.0

    @JvmField
    var PaiplainMinSat = 20

    @JvmField
    var PaiplainMinVal = 20

    @JvmField
    var PaiplainMaxBloo = 0.2

    @JvmField
    var PaiplainColBloo = 2.4

    @JvmField
    var PaiplainColRed = 0.0

    @JvmField
    var PaiplainMaxRed = 0.4

    @JvmField
    var AutoRed = true

    @JvmField
    var AutoMinBlocks = 200

    @JvmField
    var AutoResult = 1
}

class ZaPaiplain : OpenCvPipeline() {
    private fun isRed(col: DoubleArray): Boolean {
        val h = (col[0] / 255.0) * PI * 2
        val s = col[1]
        val v = col[2]

        return abs(angDiff(h, PaiplainColRed)) < PaiplainMaxRed && s > PaiplainMinSat && v > PaiplainMinVal
    }

    private fun isBlue(col: DoubleArray): Boolean {
        val h = (col[0] / 255.0) * PI * 2
        val s = col[1]
        val v = col[2]

        return abs(angDiff(h, PaiplainColBloo)) < PaiplainMaxBloo && s > PaiplainMinSat && v > PaiplainMinVal
    }

    private fun checkCol(col: DoubleArray): Boolean {
        return if (AutoRed) {
            isRed(col)
        } else {
            isBlue(col)
        }
    }

    private val frame = Mat()
    private val ff = Mat()
    override fun processFrame(input: Mat): Mat {
        if (input.empty()) {
            return input
        }
        if (DO_I_EVEN_PROCESS_FRAME) {
            input.copyTo(frame)
            Imgproc.cvtColor(frame, frame, COLOR_RGB2HSV_FULL)

            if (DRAW_BOXES || DRAW_MEDIAN) {
                frame.copyTo(ff)
            }
            var medXS = 0
            var redc = 0

            var midBlocks = 0
            var rightBlocks = 0

            run {
                val f = frame[-LUT + XOFF, -LUP + YOFF]
                log("First block", "${f[0] * PI * 2.0 / 255.0} ${f[1]} ${f[2]}")
            }

            for (cx in -LUT + XOFF..LUT + XOFF step PSTEP) {
                for (cy in -LUP + YOFF..LUP + YOFF step PSTEP) {
                    val vl = frame[cy, cx] ?: continue
                    //log ("KMSKMS", "${vl[0]} ${vl[1]} ${vl[2]}")
                    if (checkCol(vl)) {
                        if (cx < CameraMidPos) {
                            ++midBlocks
                        } else {
                            ++rightBlocks
                        }
                        medXS += cx
                        ++redc
                        if (DRAW_BOXES) {
                            Imgproc.rectangle(ff, Rect(cx, cy, PSTEP, PSTEP), Scalar(255.0, 255.0, 255.0), -1)
                        }
                    } else {
                        if (DRAW_BOXES) {
                            Imgproc.rectangle(ff, Rect(cx, cy, PSTEP, PSTEP), Scalar(
                                    max(vl[COL_INDEX] - 10.0, 0.0),
                                    max(vl[COL_INDEX] - 10.0, 0.0),
                                    max(vl[COL_INDEX] - 10.0, 0.0)), -1)
                        }
                    }
                }
            }

            lom.telemetry.addData("MidBoxes", midBlocks)
            lom.telemetry.addData("RightBoxes", rightBlocks)
            log("MidBoxes", midBlocks)
            log("RightBoxes", rightBlocks)
            AutoResult = if (midBlocks > AutoMinBlocks && rightBlocks > AutoMinBlocks) {
                if (midBlocks > rightBlocks) 1 else 2
            } else {
                if (midBlocks > AutoMinBlocks) {
                    1
                } else if (rightBlocks > AutoMinBlocks) {
                    2
                } else {
                    0
                }
            }
            if (AutoRed) {
                AutoResult = 3 - AutoResult
            }
            if (__AutoShort) {
                AutoResult = 3 - AutoResult
            }
            log("AutoResult", AutoResult)
            send_log()
            lom.telemetry.addData("GOT RESULT", AutoResult)
            lom.telemetry.update()

            val w = frame.width()
            Imgproc.line(ff, Point(CameraMidPos, 80.0), Point(CameraMidPos, 380.0), Scalar(255.0, 0.0, 0.0, 255.0), 4)

            if (DRAW_MEDIAN && !ff.empty()) {
                val c1 = Point(w / 2.0 + CUR_DONE_CORRECTION * 20, 10.0)
                val c2 = Point(w / 2.0, 10.0)

                Imgproc.line(ff, c1, c2, Scalar(100.0, if (AutoResult == 0) 0.0 else 100.0, if (AutoResult == 1) 0.0 else 100.0, 255.0), 9)
            }

            return if ((DRAW_BOXES || DRAW_MEDIAN) && !ff.empty()) {
                ff
            } else {
                frame
            }
        } else {
            return input
        }
    }
}
