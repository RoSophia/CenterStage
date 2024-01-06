package org.firstinspires.ftc.teamcode.pp

import com.qualcomm.robotcore.util.ElapsedTime
import org.apache.commons.math3.linear.Array2DRowRealMatrix
import org.apache.commons.math3.linear.DecompositionSolver
import org.apache.commons.math3.linear.LUDecomposition
import org.apache.commons.math3.linear.MatrixUtils
import org.firstinspires.ftc.teamcode.hardware.Encoder
import org.firstinspires.ftc.teamcode.utils.Pose
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.log
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.timmy
import org.firstinspires.ftc.teamcode.utils.RobotVars.USE_LOCALIZER
import org.firstinspires.ftc.teamcode.utils.RobotVars.WheelsParRDir
import org.firstinspires.ftc.teamcode.utils.RobotVars.WheelsParRName
import org.firstinspires.ftc.teamcode.utils.RobotVars.WheelsPerpPos
import org.firstinspires.ftc.teamcode.utils.RobotVars.WheelsParRPos
import org.firstinspires.ftc.teamcode.utils.RobotVars.WheelsPerpDir
import org.firstinspires.ftc.teamcode.utils.RobotVars.WheelsPerpName
import org.firstinspires.ftc.teamcode.utils.RobotVars.WheelsTicksToCm
import org.firstinspires.ftc.teamcode.utils.Util.angDiff

class TwoWheelLocalizer : Localizer {
    private var lwpos = listOf(0.0, 0.0)
    private var _pose = Pose()
    override var pose: Pose = _pose
        get() = _pose
        set(value) {
            lwpos = listOf(0.0, 0.0)
            lastHeading = 0.0
            field = value
        }

    override var poseVel: Pose = Pose()
    private var lastHeading = 0.0

    private val forwardSolver: DecompositionSolver

    private var wheelPosition = listOf(WheelsParRPos, WheelsPerpPos)
    private lateinit var encoders: List<Encoder>

    init {
        val inverseMatrix = Array2DRowRealMatrix(3, 3)
        for (i in 0..1) {
            val orientationVector = wheelPosition[i].headingVec()
            val positionVector = wheelPosition[i].vec()
            inverseMatrix.setEntry(i, 0, orientationVector.x)
            inverseMatrix.setEntry(i, 1, orientationVector.y)
            inverseMatrix.setEntry(
                    i,
                    2,
                    positionVector.x * orientationVector.y - positionVector.y * orientationVector.x
            )
        }
        inverseMatrix.setEntry(2, 2, 1.0)

        forwardSolver = LUDecomposition(inverseMatrix).solver

        require(forwardSolver.isNonSingular) { "The specified configuration cannot support full localization" }
    }

    private fun calculatePoseDelta(wheelDeltas: List<Double>, headingDelta: Double): Pose {
        val rawPoseDelta = forwardSolver.solve(
                MatrixUtils.createRealMatrix(
                        arrayOf((wheelDeltas + headingDelta).toDoubleArray())
                ).transpose()
        )
        return Pose(
                rawPoseDelta.getEntry(0, 0),
                rawPoseDelta.getEntry(1, 0),
                rawPoseDelta.getEntry(2, 0)
        )
    }


    private val thread = Thread {
        val et = ElapsedTime()
        et.reset()
        while (trunning) {
            val heading = timmy.yaw
            val wheelPositions = listOf(encoders[0].pos * WheelsTicksToCm, encoders[1].pos * WheelsTicksToCm)
            if (lwpos.isNotEmpty()) {
                val wheelDeltas = wheelPositions
                        .zip(lwpos)
                        .map { (it.first - it.second) }
                val headingDelta = angDiff(heading, lastHeading)
                val robotPoseDelta = calculatePoseDelta(wheelDeltas, headingDelta)
                _pose = relativeOdometryUpdate(_pose, robotPoseDelta)
            }

            val wheelVelocities = listOf(encoders[0].vel * WheelsTicksToCm, encoders[1].vel * WheelsTicksToCm)
            val headingVelocity = timmy.yawVel
            _pose = calculatePoseDelta(wheelVelocities, headingVelocity)

            lwpos = wheelPositions
            lastHeading = heading
            log("LocalizerRefresh", et.seconds())
            et.reset()
        }
    }
    private var trunning: Boolean = false

    override fun init(startPos: Pose) {
        if (USE_LOCALIZER) {
            encoders = listOf(
                    Encoder(WheelsPerpName, WheelsPerpDir),
                    Encoder(WheelsParRName, WheelsParRDir)
            )
        }
    }

    override fun start() {
        if (USE_LOCALIZER) {
            trunning = true
            thread.start()
        }
    }

    override fun close() {
        if (USE_LOCALIZER) {
            trunning = false
            thread.join()
        }
    }

}