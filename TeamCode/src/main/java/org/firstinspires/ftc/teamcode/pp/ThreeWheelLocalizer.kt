package org.firstinspires.ftc.teamcode.pp

import com.qualcomm.robotcore.util.ElapsedTime
import org.apache.commons.math3.linear.Array2DRowRealMatrix
import org.apache.commons.math3.linear.DecompositionSolver
import org.apache.commons.math3.linear.LUDecomposition
import org.apache.commons.math3.linear.MatrixUtils
import org.firstinspires.ftc.teamcode.hardware.Encoder
import org.firstinspires.ftc.teamcode.utils.Pose
import org.firstinspires.ftc.teamcode.utils.RobotFuncs
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.log
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.logs
import org.firstinspires.ftc.teamcode.utils.RobotVars.*
import org.firstinspires.ftc.teamcode.utils.Util.angNorm
import kotlin.math.cos
import kotlin.math.sin

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
            WheelsParRPos,
            WheelsParLPos,
            WheelsPerpPos
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

    var updatedWith = 1
    var updated = 1
    override fun update() {
        if (USE_LOCALIZER) {
            val wheelPositions = listOf(
                    encoders[0].pos * WheelsTicksToCm,
                    encoders[1].pos * WheelsTicksToCm,
                    encoders[2].pos * WheelsTicksToCm)
            /*
            logs("WheelPosParR", wheelPositions[0])
            logs("WheelPosParL", wheelPositions[1])
            logs("WheelPosPerp", wheelPositions[2])*/

            val wheelDeltas = listOf(
                    wheelPositions[0] - lwpos[0],
                    wheelPositions[1] - lwpos[1],
                    wheelPositions[2] - lwpos[2],
            )

            val robotPoseDelta = calculatePoseDelta(wheelDeltas)
            val cpose = relativeOdometryUpdate(_pose, robotPoseDelta)
            _pose = Pose(cpose.x, cpose.y, cpose.h)
            if (!timmy.localizerAccessed) {
                timmy.localizerAccessed = true
                ++updatedWith
                if (USE_IMU_LOCALIZER) {
                    _pose.h = angNorm(timmy.yaw)
                } else {
                    log("ph2", angNorm(timmy.yaw))
                    log("ph", angNorm(_pose.h))
                }
            }
            ++updated
            log("CurPos", _pose)

            val wheelVelocities = listOf(
                    encoders[0].vel * WheelsTicksToCm,
                    encoders[1].vel * WheelsTicksToCm,
                    encoders[2].vel * WheelsTicksToCm)
            poseVel = calculatePoseDelta(wheelVelocities)
            /*
                logs("WheelVelParR", wheelVelocities[0])
                logs("WheelVelParL", wheelVelocities[1])
                logs("WheelVelPerp", wheelVelocities[2])*/

            val canvas = RobotFuncs.tp.fieldOverlay()
            canvas.setStrokeWidth(1)
            canvas.setStroke("#FF00C3")
            canvas.strokeCircle(_pose.x * PP.SCALE, _pose.y * PP.SCALE, PP.robotRadius)
            canvas.setStroke("#00FFC3")
            canvas.strokeLine(_pose.x * PP.SCALE, _pose.y * PP.SCALE,
                    (_pose.x * PP.SCALE + PP.robotRadius * cos(_pose.h)), (_pose.y * PP.SCALE + PP.robotRadius * sin(_pose.h)))

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
