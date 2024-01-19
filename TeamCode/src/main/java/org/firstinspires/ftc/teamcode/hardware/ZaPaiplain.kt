package org.firstinspires.ftc.teamcode.hardware

import com.acmerobotics.dashboard.config.Config
import org.firstinspires.ftc.teamcode.hardware.CameraControls.CAMERA_UPDATE
import org.firstinspires.ftc.teamcode.hardware.CameraControls.COL_INDEX
import org.firstinspires.ftc.teamcode.hardware.CameraControls.CUR_DONE_CORRECTION
import org.firstinspires.ftc.teamcode.hardware.CameraControls.CameraMidPos
import org.firstinspires.ftc.teamcode.hardware.CameraControls.CameraRightPos
import org.firstinspires.ftc.teamcode.hardware.CameraControls.DO_I_EVEN_PROCESS_FRAME
import org.firstinspires.ftc.teamcode.hardware.CameraControls.DRAW_BOXES
import org.firstinspires.ftc.teamcode.hardware.CameraControls.DRAW_MEDIAN
import org.firstinspires.ftc.teamcode.hardware.CameraControls.INVERT
import org.firstinspires.ftc.teamcode.hardware.CameraControls.RC
import org.firstinspires.ftc.teamcode.hardware.CameraControls.TB
import org.firstinspires.ftc.teamcode.hardware.CameraControls.TR
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.log
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.send_log
import org.firstinspires.ftc.teamcode.utils.RobotVars.*
import org.firstinspires.ftc.teamcode.utils.Util.angDiff
import org.opencv.core.Core
import org.opencv.core.Mat
import org.opencv.core.Point
import org.opencv.core.Scalar
import org.opencv.imgproc.Imgproc
import org.opencv.imgproc.Imgproc.COLOR_BGR2HSV
import org.opencv.imgproc.Imgproc.COLOR_RGB2HSV
import org.openftc.easyopencv.OpenCvPipeline
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.sqrt

class Box2d(x: Int, y: Int, w: Int, h: Int) : Comparable<Box2d> {
    var sx: Int = 0
    var sy: Int = 0
    var width: Int = 0
    var height: Int = 0

    constructor() : this(0, 0, 0, 0)

    init {
        sx = x
        sy = y
        width = w
        height = h
    }

    fun gdist(): Double {
        val xd = sx.toDouble() + width.toDouble() / 2
        val yd = sy.toDouble() + height.toDouble() / 2
        return sqrt(xd * xd + yd * yd)
    }

    override fun compareTo(other: Box2d): Int {
        return if (gdist() < other.gdist()) {
            1
        } else {
            -1
        }
    }
}

@Config
object CameraControls {
    @JvmField
    var WIDTH: Int = 15

    @JvmField
    var LUP: Int = 9

    @JvmField
    var LUT: Int = 18

    @JvmField
    var XOFF: Int = 0

    @JvmField
    var YOFF: Int = 20

    @JvmField
    var INVERT: Boolean = false

    @JvmField
    var ALPHA: Double = 200.0

    @JvmField
    var TB: Double = 75.0

    @JvmField
    var TR: Double = 25.0

    @JvmField
    var COL_INDEX: Int = 1

    var RC: Double = 0.5
    var BC: Double = 1.0

    @JvmField
    var DRAW_BOXES: Boolean = false

    @JvmField
    var DRAW_MEDIAN: Boolean = true

    @JvmField
    var DO_I_EVEN_PROCESS_FRAME: Boolean = true

    @JvmField
    var CAMERA_UPDATE: Boolean = false

    @JvmField
    var CUR_DONE_CORRECTION: Int = 0

    @JvmField
    var CameraMidPos: Double = 400.0

    @JvmField
    var CameraRightPos: Double = 400.0

    @JvmField
    var PONT: Double = 200.0
}


class ZaPaiplain(width: Int, height: Int) : OpenCvPipeline() {
    var xoff: Double = 0.0

    var results: Array<Box2d> = arrayOf(Box2d())
    init {

    }

    private fun mklocations(up: Int, left: Int, width: Int, xoff: Int, yoff: Int, h: Int, w: Int): Array<Box2d> {
        val len: Int = (up * 2 - 1) * (left * 2 - 1)
        val res = Array(len) { Box2d() }
        var ci = 0
        for (i in -(up - 1) until up) {
            for (j in -(left - 1) until left) {
                res[ci] = Box2d(i * width + h / 2 - width / 2 + xoff, j * width + w / 2 - width / 2 + yoff, width, width)
                ++ci
            }
        }

        results.sort()
        return res
    }

    private fun draw(frame: Mat, cb: Box2d, vl: Double, ll: Double) {
        val p1 = Point(cb.sx.toDouble(),
                cb.sy.toDouble())
        val p3 = Point(cb.sx.toDouble() + cb.width.toDouble(),
                cb.sy.toDouble() + cb.height.toDouble())

        val red = Scalar(ll, ll, ll, CameraControls.ALPHA)
        val col = Scalar(vl, vl, vl, CameraControls.ALPHA)

        Imgproc.rectangle(frame, p1, p3, red, 7)
        Imgproc.rectangle(frame, p1, p3, col, -1)
    }

    private fun subm(img: Mat, box: Box2d): Mat {
        return img.submat(box.sy, box.sy + box.height, box.sx, box.sx + box.width)
    }

    private fun check(img: Mat, box: Box2d): DoubleArray {
        val subm = subm(img, box)
        return Core.mean(subm).`val`
    }

    private fun isRed(col: DoubleArray): Boolean {
        val h = (col[0] / 255.0) * PI * 2
        val s = col[1]
        val v = col[2]

        return abs(angDiff(h, PaiplainRed)) < PaiplainMaxRed && s > PaiplainMinSat && v > PaiplainMinVel



        val b = col[0]
        val g = col[1]
        val r = col[2]
        val rcoef = (-(r / 255) + 1.5) * RC
        val bg = b - g

        if (abs(r - g) * rcoef < TR) {
            if (bg > TB) {
                return true
            }
        }
        return false
    }

    private fun isBlue(col: DoubleArray): Boolean {
        val h = (col[0] / 255.0) * PI * 2
        val s = col[1]
        val v = col[2]

        return abs(angDiff(h, PaiplainBloo)) < PaiplainMaxBloo && s > PaiplainMinSat && v > PaiplainMinVel

        val b = col[2]
        val g = col[1]
        val r = col[0]
        return (b / 1.1 > g) && (b / 1.2 > r)
    }

    private fun checkCol(col: DoubleArray): Boolean {
        return if (AutoRed) {
            isRed(col)
        } else {
            isBlue(col)
        }
    }

    var checkLocations: Array<Box2d> = if (INVERT) {
        mklocations(CameraControls.LUP, CameraControls.LUT, CameraControls.WIDTH, CameraControls.YOFF, CameraControls.XOFF, height, width)
    } else {
        mklocations(CameraControls.LUT, CameraControls.LUP, CameraControls.WIDTH, CameraControls.XOFF, CameraControls.YOFF, width, height)
    }

    override fun processFrame(input: Mat): Mat {
        if (input.empty()) {
            return input
        }
        if (DO_I_EVEN_PROCESS_FRAME) {
            val frame = Mat()
            input.copyTo(frame)
            Imgproc.cvtColor(frame, frame, COLOR_BGR2HSV)

            val ff = Mat()
            if (DRAW_BOXES || DRAW_MEDIAN) {
                frame.copyTo(ff)
            }
            var medXS = 0
            var medYS = 0
            var redc = 0

            var midBlocks = 0
            var rightBlocks = 0

            for ((ci, element) in checkLocations.withIndex()) {
                val vl = check(frame, element)
                val cx = element.sx + element.width / 2
                val cy = element.sx + element.width / 2
                if (checkLocations.size < 9) {
                    log("MeanMid_${ci}", "${vl[0]} ${vl[1]} ${vl[2]}")
                }
                if (checkCol(vl)) {
                    if (cy < CameraMidPos) {
                        ++midBlocks
                    } else if (cy > CameraRightPos) {
                        ++rightBlocks
                    }
                    medXS += cx
                    medYS += cy
                    ++redc
                    if (DRAW_BOXES) {
                        draw(ff, element, 255.0, vl[COL_INDEX])
                    }
                } else {
                    if (DRAW_BOXES) {
                        draw(ff, element, 0.0, vl[COL_INDEX])
                    }
                }
            }
            log("MidBoxes", midBlocks)
            log("RightBoxes", rightBlocks)
            send_log()
            AutoResult = if (AutoRed) {
                if (midBlocks > AutoMinBlocksRed) {
                    1
                } else if (rightBlocks > AutoMinBlocksRed) {
                    2
                } else {
                    0
                }
            } else {
                if (midBlocks > AutoMinBlocksBlue) {
                    1
                } else if (rightBlocks > AutoMinBlocksBlue) {
                    2
                } else {
                    0
                }
            }

            val w = frame.width()
            val h = frame.height()
            val medX = if (redc > 0) {
                medXS.toDouble() / redc.toDouble()
            } else {
                w / 2.0
            }
            val medY = if (redc > 0) {
                medYS.toDouble() / redc.toDouble()
            } else {
                h / 2.0
            }

            Imgproc.line(ff, Point(CameraMidPos, 80.0), Point(CameraMidPos, 380.0), Scalar(0.0, 0.0, 0.0, 255.0), 9)
            Imgproc.line(ff, Point(CameraRightPos, 80.0), Point(CameraRightPos, 380.0), Scalar(0.0, 0.0, 0.0, 255.0), 9)

            if (DRAW_MEDIAN && !ff.empty()) {
                val p1 = Point(w / 2.0, h / 2.0)
                val p2 = Point(medX, medY)

                val c1 = Point(w / 2.0 + CUR_DONE_CORRECTION * 20, 10.0)
                val c2 = Point(w / 2.0, 10.0)

                Imgproc.line(ff, c1, c2, Scalar(0.0, 0.0, 0.0, 255.0), 9)
                if (CAMERA_UPDATE) {
                    Imgproc.line(ff, c1, c2, Scalar(0.0, 255.0, 0.0, 255.0), 8)
                } else {
                    Imgproc.line(ff, c1, c2, Scalar(200.0, 0.0, 255.0, 255.0), 8)
                }

                if (CAMERA_UPDATE) {
                    Imgproc.line(ff, p1, p2, Scalar(0.0, 0.0, 0.0, 255.0), 6)
                    Imgproc.line(ff, p1, p2, Scalar(255.0, 0.0, 0.0, 255.0), 4)
                } else {
                    Imgproc.line(ff, p1, p2, Scalar(255.0, 255.0, 255.0, 255.0), 6)
                }
            }

            xoff = medX - (w / 2.0)

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
