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
import org.firstinspires.ftc.teamcode.hardware.CameraControls.PaiplainColBloo
import org.firstinspires.ftc.teamcode.hardware.CameraControls.PaiplainMaxBloo
import org.firstinspires.ftc.teamcode.hardware.CameraControls.PaiplainMaxRed
import org.firstinspires.ftc.teamcode.hardware.CameraControls.PaiplainMinSat
import org.firstinspires.ftc.teamcode.hardware.CameraControls.PaiplainMinVal
import org.firstinspires.ftc.teamcode.hardware.CameraControls.PaiplainColRed
import org.firstinspires.ftc.teamcode.hardware.CameraControls.Squares
import org.firstinspires.ftc.teamcode.utils.DDoubleV4i
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.log
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.lom
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.send_log
import org.firstinspires.ftc.teamcode.utils.RobotVars.__AutoShort
import org.firstinspires.ftc.teamcode.utils.Util.angDiff
import org.firstinspires.ftc.teamcode.utils.Vec4i
import org.firstinspires.ftc.teamcode.utils.Vec4vi
import org.opencv.core.Mat
import org.opencv.core.Point
import org.opencv.core.Rect
import org.opencv.core.Scalar
import org.opencv.imgproc.Imgproc
import org.opencv.imgproc.Imgproc.COLOR_RGB2HSV_FULL
import org.openftc.easyopencv.OpenCvPipeline
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.max

@Config
object CameraControls {
    @JvmField
    var Squares = Vec4vi(
            DDoubleV4i(Vec4i(65, 185, 110, 80), Vec4i(405, 200, 110, 80)),
            DDoubleV4i(Vec4i(30, 160, 110, 80), Vec4i(360, 178, 110, 80)),
            DDoubleV4i(Vec4i(10, 170, 110, 80), Vec4i(340, 175, 110, 80)),
            DDoubleV4i(Vec4i(75, 190, 110, 80), Vec4i(410, 210, 110, 80)),
    )

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
    var CameraMidPos: Double = 200.0

    @JvmField
    var PaiplainMinSat = 90

    @JvmField
    var PaiplainMinVal = 40

    @JvmField
    var PaiplainMaxBloo = 0.8

    @JvmField
    var PaiplainColBloo = 3.9

    @JvmField
    var PaiplainColRed = 0.0

    @JvmField
    var PaiplainMaxRed = 0.6

    @JvmField
    var AutoRed = true

    @JvmField
    var AutoMinBlocks = 3000

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
    private fun getr(midBlocks: Int, rightBlocks: Int): Int {
        return if (midBlocks > AutoMinBlocks && rightBlocks > AutoMinBlocks) {
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
    }

    private fun chk(p: Vec4i): Int {
        var res = 0
        for (x in p.a .. p.a + p.c) {
            for (y in p.b .. p.b + p.d) {
                val vl = frame[y, x] ?: continue
                if (checkCol(vl)) {
                    Imgproc.rectangle(ff, Rect(x, y, 1, 1), Scalar(255.0, 255.0, 255.0), -1)
                    ++res
                } else {
                    if (DRAW_BOXES) {
                        Imgproc.rectangle(ff, Rect(x, y, 1, 1), Scalar(
                                max(vl[COL_INDEX] - 10.0, 0.0),
                                max(vl[COL_INDEX] - 10.0, 0.0),
                                max(vl[COL_INDEX] - 10.0, 0.0)), -1)
                    }
                }
            }
        }
        return res
    }

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
            /*
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
            val curr = getr(midBlocks, rightBlocks)
             */

            val cc = (if (AutoRed) 1 else 0) + (if (__AutoShort) 2 else 0)
            val left = chk(Squares[cc].left)
            val right = chk(Squares[cc].right)
            log("leftDetections", left)
            log("rightDetections", right)
            log("cc", cc)
            val curr = getr(left, right)
            val results = listOf(
                    listOf(0, 1, 2),
                    listOf(0, 2, 1),
                    listOf(0, 2, 1),
                    listOf(0, 1, 2))
            AutoResult = results[cc][curr]
            log("AutoResult", AutoResult)
            send_log()
            lom.telemetry.addData("Detections Right", right)
            lom.telemetry.addData("Detections Left", left)
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
