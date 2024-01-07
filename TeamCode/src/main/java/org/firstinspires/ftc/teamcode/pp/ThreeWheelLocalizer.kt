package org.firstinspires.ftc.teamcode.pp

import com.qualcomm.robotcore.util.ElapsedTime
import org.apache.commons.math3.linear.Array2DRowRealMatrix
import org.apache.commons.math3.linear.DecompositionSolver
import org.apache.commons.math3.linear.LUDecomposition
import org.apache.commons.math3.linear.MatrixUtils
import org.firstinspires.ftc.teamcode.hardware.Encoder
import org.firstinspires.ftc.teamcode.utils.Pose
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.timmy
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.log
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.logs
import org.firstinspires.ftc.teamcode.utils.RobotVars.*

class ThreeWheelLocalizer : Localizer {
    private var lwpos = listOf(0.0, 0.0, 0.0)
    private var _pose = Pose()
    override var pose: Pose = _pose
        get() = _pose
        set(value) {
            lwpos = listOf(0.0, 0.0, 0.0)
            lastHeading = 0.0
            field = value
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

    /*
    //private val thread = Thread {
        val et = ElapsedTime()
        et.reset()
        while (trunning) {
            val wheelPositions = listOf(
                    encoders[0].pos * WheelsTicksToCm,
                    encoders[1].pos * WheelsTicksToCm,
                    encoders[2].pos * WheelsTicksToCm)
            logs("WheelPosParR", wheelPositions[0])
            logs("WheelPosParL", wheelPositions[1])
            logs("WheelPosPerp", wheelPositions[2])

            val wheelDeltas = listOf(
                    wheelPositions[0] - lwpos[0],
                    wheelPositions[1] - lwpos[1],
                    wheelPositions[2] - lwpos[2],
            )
            val heading = timmy.yaw

            val robotPoseDelta = calculatePoseDelta(wheelDeltas)
            val cpose = relativeOdometryUpdate(_pose, robotPoseDelta)
            _pose = Pose(cpose.x, cpose.y, heading)

            val wheelVelocities = listOf(
                    encoders[0].vel * WheelsTicksToCm,
                    encoders[1].vel * WheelsTicksToCm,
                    encoders[2].vel * WheelsTicksToCm)
            poseVel = calculatePoseDelta(wheelVelocities)
            logs("WheelVelParR", wheelVelocities[0])
            logs("WheelVelParL", wheelVelocities[1])
            logs("WheelVelPerp", wheelVelocities[2])

            lwpos = wheelPositions
            logs("LocalizerRefresh", et.seconds())
            et.reset()
        }
    }*/
    private var trunning: Boolean = false

    val et = ElapsedTime()
    override fun update() {
        val wheelPositions = listOf(
                encoders[0].pos * WheelsTicksToCm,
                encoders[1].pos * WheelsTicksToCm,
                encoders[2].pos * WheelsTicksToCm)
        logs("WheelPosParR", wheelPositions[0])
        logs("WheelPosParL", wheelPositions[1])
        logs("WheelPosPerp", wheelPositions[2])

        val wheelDeltas = listOf(
                wheelPositions[0] - lwpos[0],
                wheelPositions[1] - lwpos[1],
                wheelPositions[2] - lwpos[2],
        )
        val heading = timmy.yaw

        val robotPoseDelta = calculatePoseDelta(wheelDeltas)
        val cpose = relativeOdometryUpdate(_pose, robotPoseDelta)
        _pose = Pose(cpose.x, cpose.y, heading)

        val wheelVelocities = listOf(
                encoders[0].vel * WheelsTicksToCm,
                encoders[1].vel * WheelsTicksToCm,
                encoders[2].vel * WheelsTicksToCm)
        poseVel = calculatePoseDelta(wheelVelocities)
        logs("WheelVelParR", wheelVelocities[0])
        logs("WheelVelParL", wheelVelocities[1])
        logs("WheelVelPerp", wheelVelocities[2])

        lwpos = wheelPositions
        logs("LocalizerRefresh", et.seconds())
        et.reset()
    }

    override fun init(startPos: Pose) {
        if (USE_LOCALIZER) {
            encoders = listOf(
                    Encoder(WheelsParLName, WheelsParLDir),
                    Encoder(WheelsParRName, WheelsParRDir),
                    Encoder(WheelsPerpName, WheelsPerpDir)
            )
            et.reset()
        }
    }

    /*
    override fun start() {
        if (USE_LOCALIZER) {
            trunning = true
            thread.start()
        }
    }

     */

    /*
    override fun close() {
        if (USE_LOCALIZER) {
            trunning = false
            thread.join()
        }
    }

     */
}
