package org.firstinspires.ftc.teamcode.pp

import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.hardware.Encoder
import org.firstinspires.ftc.teamcode.utils.Pose
import org.firstinspires.ftc.teamcode.utils.RobotVars.FER
import org.firstinspires.ftc.teamcode.utils.RobotVars.FES
import org.firstinspires.ftc.teamcode.utils.RobotVars.LER
import org.firstinspires.ftc.teamcode.utils.RobotVars.LES
import org.firstinspires.ftc.teamcode.utils.RobotVars.RER
import org.firstinspires.ftc.teamcode.utils.RobotVars.RES
import java.util.Arrays

class ThreeWheelLocalizer : Localizer {
    var TICKS_PER_REV = 8192.0
    var WHEEL_RADIUS = 1.75
    var GEAR_RATIO = 1.0 // output (wheel) speed / input (encoder) speed
    var LATERAL_DISTANCE = 11.65 // distance between the left and right wheels
    var FORWARD_OFFSET = 26.15 // offset of the lateral wheel

    var wheelPoses = listOf(
            Pose(FORWARD_OFFSET / 2, LATERAL_DISTANCE / 2, 0.0),  // left
            Pose(FORWARD_OFFSET / 2, -LATERAL_DISTANCE / 2, 0.0),  // right
            Pose(-FORWARD_OFFSET / 2, LATERAL_DISTANCE / 2, Math.toRadians(90.0))) // front

    /*val leftEncoder = Encoder("LE")
    val rightEncoder = Encoder("RE")
    val frontEncoder = Encoder("FE")*/

    var lwpos = listOf(0, 0, 0)
    override var pose = Pose()
        set(v) {
            field = v
            lwpos = listOf(0, 0, 0)
        }

    override var poseVel = Pose()

    override fun init() {}
}

/*
private val forwardSolver: DecompositionSolver

fun StandardTrackingWheelLocalizer(hardwareMap: HardwareMap) {
    leftEncoder = Encoder(hardwareMap[DcMotorEx::class.java, LES])
    rightEncoder = Encoder(hardwareMap[DcMotorEx::class.java, RES])
    frontEncoder = Encoder(hardwareMap[DcMotorEx::class.java, FES])
    leftEncoder.setDirection(if (LER) Encoder.Direction.REVERSE else Encoder.Direction.FORWARD)
    rightEncoder.setDirection(if (RER) Encoder.Direction.REVERSE else Encoder.Direction.FORWARD)
    frontEncoder.setDirection(if (FER) Encoder.Direction.REVERSE else Encoder.Direction.FORWARD)

    // TODO: reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)
}

fun encoderTicksToInches(ticks: Double): Double {
    return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV
}

init {
    val inverseMatrix = Array2DRowRealMatrix(3, 3)
    for (i in 0..2) {
        val orientationVector = wheelPoses[i].headingVec()
        val positionVector = wheelPoses[i].vec()
        inverseMatrix.setEntry(i, 0, orientationVector.x)
        inverseMatrix.setEntry(i, 1, orientationVector.y)
        inverseMatrix.setEntry(
                i,
                2,
                positionVector.x * orientationVector.y - positionVector.y * orientationVector.x
        )
    }

    forwardSolver = LUDecomposition(inverseMatrix).solver

    require(forwardSolver.isNonSingular) { "The specified configuration cannot support full localization" }
}

private fun calculatePoseDelta(wheelDeltas: List<Double>): Pose2d {
    val rawPoseDelta = forwardSolver.solve(
            MatrixUtils.createRealMatrix(
                    arrayOf(wheelDeltas.toDoubleArray())
            ).transpose()
    )
    return Pose2d(
            rawPoseDelta.getEntry(0, 0),
            rawPoseDelta.getEntry(1, 0),
            rawPoseDelta.getEntry(2, 0)
    )
}

override fun update() {
    val wheelPositions = getWheelPositions()
    if (lastWheelPositions.isNotEmpty()) {
        val wheelDeltas = wheelPositions
                .zip(lastWheelPositions)
                .map { it.first - it.second }
        val robotPoseDelta = calculatePoseDelta(wheelDeltas)
        _poseEstimate = Kinematics.relativeOdometryUpdate(_poseEstimate, robotPoseDelta)
    }

    val wheelVelocities = getWheelVelocities()
    if (wheelVelocities != null) {
        poseVelocity = calculatePoseDelta(wheelVelocities)
    }

    lastWheelPositions = wheelPositions
}*/

//abstract fun getWheelPositions(): List<Double>
