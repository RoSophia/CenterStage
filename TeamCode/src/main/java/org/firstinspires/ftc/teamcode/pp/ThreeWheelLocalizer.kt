package org.firstinspires.ftc.teamcode.pp

import com.qualcomm.robotcore.util.ElapsedTime
import org.apache.commons.math3.linear.Array2DRowRealMatrix
import org.apache.commons.math3.linear.DecompositionSolver
import org.apache.commons.math3.linear.LUDecomposition
import org.apache.commons.math3.linear.MatrixUtils
import org.firstinspires.ftc.teamcode.hardware.Encoder
import org.firstinspires.ftc.teamcode.utils.Pose
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.log
import org.firstinspires.ftc.teamcode.utils.RobotVars.*
import org.firstinspires.ftc.teamcode.utils.Util.angDiff
import org.firstinspires.ftc.teamcode.utils.Util.angNorm

class ThreeWheelLocalizer : Localizer {
    private var lwpos = listOf(0.0, 0.0, 0.0)
    private var _pose = Pose()
    override var pose: Pose = _pose
        get() = _pose
        set(v) {
            lwpos = listOf(0.0, 0.0, 0.0)
            lastHeading = v.h
            field = v
        }

    override var poseVel: Pose = Pose()
    private var lastHeading = 0.0

    private val forwardSolver: DecompositionSolver

    private var wheelPositions = listOf(
            WheelsParRPos + WheelsAdjPose,
            WheelsParLPos + WheelsAdjPose,
            WheelsPerpPos + WheelsAdjPose
    )
    private lateinit var encoders: List<Encoder>

    init {
        val inverseMatrix = Array2DRowRealMatrix(3, 3)
        for (i in 0..2) {
            val orientationVector = wheelPositions[i].headingVec()
            val positionVector = wheelPositions[i].vec()
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

    private fun calculatePoseDelta(wheelDeltas: List<Double>): Pose {
        val ma = MatrixUtils.createRealMatrix(
                arrayOf(wheelDeltas.toDoubleArray())
        ).transpose()

        val rawPoseDelta = forwardSolver.solve(ma)
        return Pose(
                rawPoseDelta.getEntry(0, 0),
                rawPoseDelta.getEntry(1, 0),
                rawPoseDelta.getEntry(2, 0))
    }

    val vet = ElapsedTime()
    override fun update() {
        if (USE_LOCALIZER) {
            val wheelPositions = listOf(
                    encoders[0].pos * WheelsParTicksToCm,
                    encoders[1].pos * WheelsParTicksToCm,
                    encoders[2].pos * WheelsParTicksToCm)

            val wheelDeltas = listOf(
                    wheelPositions[0] - lwpos[0],
                    wheelPositions[1] - lwpos[1],
                    wheelPositions[2] - lwpos[2],
            )

            val robotPoseDelta = calculatePoseDelta(wheelDeltas)
            val cpose = relativeOdometryUpdate(_pose, robotPoseDelta)
            _pose = Pose(cpose.x, cpose.y, cpose.h)
            if (USE_COMBINED_LOCALIZER) {
                if (!timmy.localizerAccessed) {
                    timmy.localizerAccessed = true
                    log("timmyVel", angDiff(angNorm(_pose.h), angNorm(timmy.yaw)) / vet.seconds())
                    vet.reset()
                    _pose.h = angNorm(timmy.yaw)
                    log("timmyyaw", _pose.h)
                }
            } else {
                _pose.h = angNorm(timmy.yaw)
            }
            log("CurPos", _pose)

            if (__COIN) {
                val wheelVelocities = listOf(
                        encoders[0].vel * WheelsParTicksToCm,
                        encoders[1].vel * WheelsParTicksToCm,
                        encoders[2].vel * WheelsParTicksToCm)
                log("WheelVelParR", wheelVelocities[0])
                log("WheelVelParL", wheelVelocities[1])
                log("WheelVelPerp", wheelVelocities[2])
            }

            lwpos = wheelPositions
        }
    }

    override fun init(startPos: Pose) {
        pose = startPos
        if (USE_LOCALIZER) {
            encoders = listOf(
                    Encoder(WheelsParLName, WheelsParLDir),
                    Encoder(WheelsParRName, WheelsParRDir),
                    Encoder(WheelsPerpName, WheelsPerpDir)
            )
        }
    }
}
