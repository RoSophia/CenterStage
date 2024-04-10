package org.firstinspires.ftc.teamcode.hardware

import org.firstinspires.ftc.teamcode.utils.RobotVars.QrDecimation
import org.firstinspires.ftc.teamcode.utils.RobotVars.QrThreads
import org.opencv.calib3d.Calib3d
import org.opencv.core.CvType
import org.opencv.core.Mat
import org.opencv.core.MatOfDouble
import org.opencv.core.MatOfPoint2f
import org.opencv.core.MatOfPoint3f
import org.opencv.core.Point
import org.opencv.core.Point3
import org.opencv.core.Scalar
import org.opencv.imgproc.Imgproc
import org.openftc.apriltag.AprilTagDetection
import org.openftc.apriltag.AprilTagDetectorJNI
import org.openftc.easyopencv.OpenCvPipeline

// UNITS ARE METERS
class QrDetector(
        private var tagsize: Double,
        private val fx: Double,
        private val fy: Double,
        private val cx: Double,
        private val cy: Double) : OpenCvPipeline() {
    private var nativeApriltagPtr: Long
    private val grey = Mat()
    private var latestDetections = ArrayList<AprilTagDetection>()
    private var detectionsUpdate: ArrayList<AprilTagDetection>? = ArrayList()
    private val detectionsUpdateSync = Any()
    private var cameraMatrix: Mat? = null
    private var blue = Scalar(7.0, 197.0, 235.0, 255.0)
    private var red = Scalar(255.0, 0.0, 0.0, 255.0)
    private var green = Scalar(0.0, 255.0, 0.0, 255.0)
    private var white = Scalar(255.0, 255.0, 255.0, 255.0)
    private var decimation = QrDecimation
    private var needToSetDecimation = false
    private val decimationSync = Any()

    init {
        constructMatrix()

        // Allocate a native context object. See the corresponding deletion in the finalizer
        nativeApriltagPtr = AprilTagDetectorJNI.createApriltagDetector(AprilTagDetectorJNI.TagFamily.TAG_36h11.string, 3f, QrThreads)
    }

    fun finalize() {
        // Might be null if createApriltagDetector() threw an exception
        if (nativeApriltagPtr != 0L) {
            // Delete the native context we created in the constructor
            AprilTagDetectorJNI.releaseApriltagDetector(nativeApriltagPtr)
            nativeApriltagPtr = 0
        } else {
            println("AprilTagDetectionPipeline.finalize(): nativeApriltagPtr was NULL")
        }
    }

    override fun processFrame(input: Mat): Mat {
        // Convert to greyscale
        Imgproc.cvtColor(input, grey, Imgproc.COLOR_RGBA2GRAY)
        synchronized(decimationSync) {
            if (needToSetDecimation) {
                AprilTagDetectorJNI.setApriltagDetectorDecimation(nativeApriltagPtr, decimation)
                needToSetDecimation = false
            }
        }

        // Run AprilTag
        latestDetections = AprilTagDetectorJNI.runAprilTagDetectorSimple(nativeApriltagPtr, grey, tagsize, fx, fy, cx, cy)
        synchronized(detectionsUpdateSync) { detectionsUpdate = latestDetections }

        // For fun, use OpenCV to draw 6DOF markers on the image. We actually recompute the pose using
        // OpenCV because I haven't yet figured out how to re-use AprilTag's pose in OpenCV.
        for (detection in latestDetections) {
            val pose = poseFromTrapezoid(detection.corners, cameraMatrix, tagsize, tagsize)
            drawAxisMarker(input, tagsize / 2.0, 6, pose.rvec, pose.tvec, cameraMatrix)
            draw3dCubeMarker(input, tagsize, tagsize, tagsize, 5, pose.rvec, pose.tvec, cameraMatrix)
        }
        return input
    }

    fun setDecimation(decimation: Float) {
        synchronized(decimationSync) {
            this.decimation = decimation
            needToSetDecimation = true
        }
    }

    fun getDetectionsUpdate(): ArrayList<AprilTagDetection>? {
        synchronized(detectionsUpdateSync) {
            val ret = detectionsUpdate
            detectionsUpdate = null
            return ret
        }
    }

    private fun constructMatrix() {
        //     Construct the camera matrix.
        //
        //      --         --
        //     | fx   0   cx |
        //     | 0    fy  cy |
        //     | 0    0   1  |
        //      --         --
        //
        cameraMatrix = Mat(3, 3, CvType.CV_32FC1)
        cameraMatrix!!.put(0, 0, fx)
        cameraMatrix!!.put(0, 1, 0.0)
        cameraMatrix!!.put(0, 2, cx)
        cameraMatrix!!.put(1, 0, 0.0)
        cameraMatrix!!.put(1, 1, fy)
        cameraMatrix!!.put(1, 2, cy)
        cameraMatrix!!.put(2, 0, 0.0)
        cameraMatrix!!.put(2, 1, 0.0)
        cameraMatrix!!.put(2, 2, 1.0)
    }

    /**
     * Draw a 3D axis marker on a detection. (Similar to what Vuforia does)
     *
     * @param buf          the RGB buffer on which to draw the marker
     * @param length       the length of each of the marker 'poles'
     * @param rvec         the rotation vector of the detection
     * @param tvec         the translation vector of the detection
     * @param cameraMatrix the camera matrix used when finding the detection
     */
    private fun drawAxisMarker(buf: Mat?, length: Double, thickness: Int, rvec: Mat?, tvec: Mat?, cameraMatrix: Mat?) {
        // The points in 3D space we wish to project onto the 2D image plane.
        // The origin of the coordinate space is assumed to be in the center of the detection.
        val axis = MatOfPoint3f(
                Point3(0.0, 0.0, 0.0),
                Point3(length, 0.0, 0.0),
                Point3(0.0, length, 0.0),
                Point3(0.0, 0.0, -length)
        )

        // Project those points
        val matProjectedPoints = MatOfPoint2f()
        Calib3d.projectPoints(axis, rvec, tvec, cameraMatrix, MatOfDouble(), matProjectedPoints)
        val projectedPoints = matProjectedPoints.toArray()

        // Draw the marker!
        Imgproc.line(buf, projectedPoints[0], projectedPoints[1], red, thickness)
        Imgproc.line(buf, projectedPoints[0], projectedPoints[2], green, thickness)
        Imgproc.line(buf, projectedPoints[0], projectedPoints[3], blue, thickness)
        Imgproc.circle(buf, projectedPoints[0], thickness, white, -1)
    }

    private fun draw3dCubeMarker(buf: Mat?, length: Double, tagWidth: Double, tagHeight: Double, thickness: Int, rvec: Mat?, tvec: Mat?, cameraMatrix: Mat?) {
        //axis = np.float32([[0,0,0], [0,3,0], [3,3,0], [3,0,0],
        //       [0,0,-3],[0,3,-3],[3,3,-3],[3,0,-3] ])

        // The points in 3D space we wish to project onto the 2D image plane.
        // The origin of the coordinate space is assumed to be in the center of the detection.
        val axis = MatOfPoint3f(
                Point3(-tagWidth / 2, tagHeight / 2, 0.0),
                Point3(tagWidth / 2, tagHeight / 2, 0.0),
                Point3(tagWidth / 2, -tagHeight / 2, 0.0),
                Point3(-tagWidth / 2, -tagHeight / 2, 0.0),
                Point3(-tagWidth / 2, tagHeight / 2, -length),
                Point3(tagWidth / 2, tagHeight / 2, -length),
                Point3(tagWidth / 2, -tagHeight / 2, -length),
                Point3(-tagWidth / 2, -tagHeight / 2, -length))

        // Project those points
        val matProjectedPoints = MatOfPoint2f()
        Calib3d.projectPoints(axis, rvec, tvec, cameraMatrix, MatOfDouble(), matProjectedPoints)
        val projectedPoints = matProjectedPoints.toArray()

        // Pillars
        for (i in 0..3) {
            Imgproc.line(buf, projectedPoints[i], projectedPoints[i + 4], blue, thickness)
        }

        // Base lines
        //Imgproc.line(buf, projectedPoints[0], projectedPoints[1], blue, thickness);
        //Imgproc.line(buf, projectedPoints[1], projectedPoints[2], blue, thickness);
        //Imgproc.line(buf, projectedPoints[2], projectedPoints[3], blue, thickness);
        //Imgproc.line(buf, projectedPoints[3], projectedPoints[0], blue, thickness);

        // Top lines
        Imgproc.line(buf, projectedPoints[4], projectedPoints[5], green, thickness)
        Imgproc.line(buf, projectedPoints[5], projectedPoints[6], green, thickness)
        Imgproc.line(buf, projectedPoints[6], projectedPoints[7], green, thickness)
        Imgproc.line(buf, projectedPoints[4], projectedPoints[7], green, thickness)
    }

    /**
     * Extracts 6DOF pose from a trapezoid, using a camera intrinsics matrix and the
     * original size of the tag.
     *
     * @param points       the points which form the trapezoid
     * @param cameraMatrix the camera intrinsics matrix
     * @param tagsizeX     the original width of the tag
     * @param tagsizeY     the original height of the tag
     * @return the 6DOF pose of the camera relative to the tag
     */
    private fun poseFromTrapezoid(points: Array<Point?>, cameraMatrix: Mat?, tagsizeX: Double, tagsizeY: Double): QRPose {
        // The actual 2d points of the tag detected in the image
        val points2d = MatOfPoint2f(*points)

        // The 3d points of the tag in an 'ideal projection'
        val arrayPoints3d = arrayOfNulls<Point3>(4)
        arrayPoints3d[0] = Point3(-tagsizeX / 2, tagsizeY / 2, 0.0)
        arrayPoints3d[1] = Point3(tagsizeX / 2, tagsizeY / 2, 0.0)
        arrayPoints3d[2] = Point3(tagsizeX / 2, -tagsizeY / 2, 0.0)
        arrayPoints3d[3] = Point3(-tagsizeX / 2, -tagsizeY / 2, 0.0)
        val points3d = MatOfPoint3f(*arrayPoints3d)

        // Using this information, actually solve for pose
        val cp = QRPose()
        Calib3d.solvePnP(points3d, points2d, cameraMatrix, MatOfDouble(), cp.rvec, cp.tvec, false)
        return cp
    }

    /*
     * A simple container to hold both rotation and translation
     * vectors, which together form a 6DOF pose.
     */
    internal inner class QRPose {
        var rvec: Mat
        var tvec: Mat

        constructor() {
            rvec = Mat()
            tvec = Mat()
        }

        constructor(rvec: Mat, tvec: Mat) {
            this.rvec = rvec
            this.tvec = tvec
        }
    }
}