package org.firstinspires.ftc.teamcode.pp

import com.qualcomm.robotcore.util.ElapsedTime
import org.apache.commons.math3.linear.Array2DRowRealMatrix
import org.apache.commons.math3.linear.DecompositionSolver
import org.apache.commons.math3.linear.LUDecomposition
import org.apache.commons.math3.linear.MatrixUtils
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.WhiteBalanceControl
import org.firstinspires.ftc.teamcode.hardware.Encoder
import org.firstinspires.ftc.teamcode.utils.Pose
import org.firstinspires.ftc.teamcode.utils.RobotFuncs
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.aprilTag
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.log
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.logs
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.logsst
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.logst
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.orp
import org.firstinspires.ftc.teamcode.utils.RobotVars.*
import org.firstinspires.ftc.teamcode.utils.Util.angNorm
import org.firstinspires.ftc.teamcode.utils.Vec6Gen
import kotlin.math.PI

class MinciunaMeaDinAprilie(
        @JvmField var centerToCam: Pose,
        var aprilPoses: Vec6Gen<Pose>,
        var kms: Vec6Gen<Boolean>) {
    constructor(centerToCam: Pose) : this (centerToCam, Vec6Gen(Pose(),Pose(),Pose(),Pose(),Pose(),Pose()), Vec6Gen(a = false, b = false, c = false, d = false, e = false, f = false))
    fun reset() { kms = Vec6Gen(a = true, b = true, c = true, d = true, e = true, f = true) }
}

class ThreeWheelLocalizer : Localizer {
    private var lwpos = listOf(0.0, 0.0, 0.0)
    override var poseVel = Pose()
    private var _pose = Pose()
    override var pose: Pose = _pose
        get() = _pose
        set(v) { field = v; _pose = v; }

    private val forwardSolver: DecompositionSolver
    private var wheelPositions = listOf(WheelsParRPos + WheelsAdjPose, WheelsParLPos + WheelsAdjPose, WheelsPerpPos + WheelsAdjPose)
    private lateinit var encoders: List<Encoder>

    init { AprilApril.reset(); val inverseMatrix = Array2DRowRealMatrix(3, 3); for (i in 0..2) { val orientationVector = wheelPositions[i].headingVec(); val positionVector = wheelPositions[i].vec(); inverseMatrix.setEntry(i, 0, orientationVector.x); inverseMatrix.setEntry(i, 1, orientationVector.y); inverseMatrix.setEntry(i, 2, positionVector.x * orientationVector.y - positionVector.y * orientationVector.x) }; forwardSolver = LUDecomposition(inverseMatrix).solver; require(forwardSolver.isNonSingular) { "The specified configuration cannot support full localization" } }

    private fun calculatePoseDelta(wheelDeltas: List<Double>): Pose { val ma = MatrixUtils.createRealMatrix(arrayOf(wheelDeltas.toDoubleArray())).transpose(); val rawPoseDelta = forwardSolver.solve(ma); return Pose(rawPoseDelta.getEntry(0, 0), rawPoseDelta.getEntry(1, 0), rawPoseDelta.getEntry(2, 0)) }

    private class TimedPose(val t: Long, val p: Pose, val v: Pose) { override fun toString() = "$p($v) @ $t" }

    private val posDeque = ArrayDeque<TimedPose>()

    private fun lerp(p1: Pose, p2: Pose, a: Double) = p1 + (p2 - p1) * a

    private fun getpose(t: Long): Pose {
        for (i in 1 until posDeque.size) {
            if (posDeque[i].t >= t) {
                val at = posDeque[i]
                val pt = posDeque[i - 1]
                return lerp(pt.p, at.p, ((t - pt.t) / 1e9) / ((at.t - pt.t) / 1e9))
            }
        }
        return pose
    }

    private var zap = Pose()

    private fun parseDetections() {
        var resPose = Pose(); var adds = 0; var gettime = 0L; val cclock = System.nanoTime()
        posDeque.addLast(TimedPose(cclock, pose.duplicate(), poseVel.duplicate()))
        while (posDeque.isNotEmpty() && posDeque.first().t < cclock - AprilMaxRetentionTime) { posDeque.removeFirst() }

        val detects = aprilTag?.freshDetections ?: return /// KYS KYS KYS KYS I LOVE KOTLIN
        for (det in detects) {
            val cid = det.id - 1
            if (cid in 0..5) {
                gettime = det.frameAcquisitionNanoTime
                val thenp = getpose(gettime)
                val correctedDist = orp(det.ftcPose, thenp.h)
                if (AprilApril.kms[cid]) {
                    AprilApril.kms[cid] = false; AprilApril.aprilPoses[cid] = getpose(gettime) + correctedDist
                    logs("Difference from time for $cid is ", thenp - pose)
                    logs("Setting tag $cid to ", AprilApril.aprilPoses[cid])
                } else {
                    ++adds
                    resPose += AprilApril.aprilPoses[cid] - correctedDist
                    logsst("Got from $cid dist to cam, ${AprilApril.aprilPoses[cid] - correctedDist}")
                    logsst("Got from $cid dist to centre, ${AprilApril.aprilPoses[cid] - correctedDist}")
                }

                logs("Tagdist $cid", String.format("%.2f %.2f %.2f %.2f", det.ftcPose.y * 2.54, -det.ftcPose.z * 2.54, det.ftcPose.x * 2.54, det.ftcPose.yaw * PI / 180))
                logs("CorrectedDist $cid", String.format("%.2f %.2f %.2f", correctedDist.x, correctedDist.y, correctedDist.h))
                logs("Turn angle ", String.format("%.2f", angNorm(thenp.h * (KMSIMU * timmy.yaw + KMSKMSKMS * PI))))
            }
        }
        if (!AprilApril.kms[0] && !AprilApril.kms[1]) { logs("D01", AprilApril.aprilPoses[1] - AprilApril.aprilPoses[0]) }
        if (!AprilApril.kms[1] && !AprilApril.kms[2]) { logs("D12", AprilApril.aprilPoses[2] - AprilApril.aprilPoses[1]) }
        if (!AprilApril.kms[0] && !AprilApril.kms[2]) { logs("D02", AprilApril.aprilPoses[2] - AprilApril.aprilPoses[0]) }
        __DETECTIONS = adds
        if (adds > 1) {
            resPose /= adds

            zap += resPose - _pose
            //_pose.x = resPose.x
            //_pose.y = resPose.y

            /*
            for (i in 0 until posDeque.size - 1) {
                if (posDeque[i].t >= gettime) {
                    resPose += posDeque[i].v * ((posDeque[i + 1].t - posDeque[i].t).toDouble() / 5e9)
                }
            }
            logs("Wanted correction diff", resPose - pose)*/
        }
        log("zazap", zap)
    }

    override fun update() {
        if (USE_LOCALIZER) {
            val wheelPositions = listOf(encoders[0].pos * WheelsParTicksToCm, encoders[1].pos * WheelsParTicksToCm, encoders[2].pos * WheelsParTicksToCm)
            val wheelDeltas = listOf(wheelPositions[0] - lwpos[0], wheelPositions[1] - lwpos[1], wheelPositions[2] - lwpos[2])

            _pose = relativeOdometryUpdate(_pose, calculatePoseDelta(wheelDeltas))
            if (!timmy.localizerAccessed || !USE_COMBINED_LOCALIZER) { timmy.localizerAccessed = true; _pose.h = timmy.yaw }

            val wheelVelocities = listOf(encoders[0].vel * WheelsParTicksToCm, encoders[1].vel * WheelsParTicksToCm, encoders[2].vel * WheelsParTicksToCm)
            poseVel = calculatePoseDelta(wheelVelocities)

            if (USE_CAMERA_DETECTION) { parseDetections() }

            lwpos = wheelPositions

            log("CurPos", _pose)
            if (__COIN) { log("WheelVelParR", wheelVelocities[0]); log("WheelVelParL", wheelVelocities[1]); log("WheelVelPerp", wheelVelocities[2]) }
        }
    }

    override fun init(startPos: Pose) { pose = startPos; if (USE_LOCALIZER) { encoders = listOf( Encoder(WheelsParLName, WheelsParLDir), Encoder(WheelsParRName, WheelsParRDir), Encoder(WheelsPerpName, WheelsPerpDir) ) } }
}