package org.firstinspires.ftc.teamcode.pp

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.utils.Pose
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.hardwareMap
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.log
import org.firstinspires.ftc.teamcode.utils.RobotVars.USE_COMBINED_LOCALIZER
import org.firstinspires.ftc.teamcode.utils.RobotVars.USE_IMU_LOCALIZER
import org.firstinspires.ftc.teamcode.utils.RobotVars.WheelsAdjPose
import org.firstinspires.ftc.teamcode.utils.RobotVars.WheelsParLName
import org.firstinspires.ftc.teamcode.utils.RobotVars.WheelsParLPos
import org.firstinspires.ftc.teamcode.utils.RobotVars.WheelsParRName
import org.firstinspires.ftc.teamcode.utils.RobotVars.WheelsParRPos
import org.firstinspires.ftc.teamcode.utils.RobotVars.WheelsPerpName
import org.firstinspires.ftc.teamcode.utils.RobotVars.WheelsPerpPos
import org.firstinspires.ftc.teamcode.utils.RobotVars.WheelsParTicksToCm
import org.firstinspires.ftc.teamcode.utils.RobotVars.WheelsPerpTicksToCm
import org.firstinspires.ftc.teamcode.utils.RobotVars.__COIN
import org.firstinspires.ftc.teamcode.utils.RobotVars.timmy
import org.firstinspires.ftc.teamcode.utils.Util.angNorm
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.sin

class PeruEncoder(private val motor: DcMotor) {
    var reversed: Boolean = false

    init {
        motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
    }

    fun getPosition(): Int = motor.currentPosition * (if (reversed) 1 else -1)
}

class PeruWheelLocalizer : Localizer {
    private val FORWARD_TICKS_PER_CM: Double = 223863.0 / 300.0
    private val LATERAL_TICKS_PER_CM: Double = 222970.0 / 300.0

    //offsets in cm from center of rotation,
    //y coord for parallels,
    //x coord for perpendicular
    private val LEFT_PARALLEL_OFFSET: Double = (WheelsParLPos + WheelsAdjPose).y
    private val RIGHT_PARALLEL_OFFSET: Double = (WheelsParRPos + WheelsAdjPose).y
    private val PERPENDICULAR_OFFSET: Double = (WheelsPerpPos + WheelsAdjPose).x

    private val leftParallelEncoder = PeruEncoder(hardwareMap.dcMotor.get(WheelsParLName))
    private val rightParallelEncoder = PeruEncoder(hardwareMap.dcMotor.get(WheelsParRName))
    private val perpendicularEncoder = PeruEncoder(hardwareMap.dcMotor.get(WheelsPerpName))

    init {
        leftParallelEncoder.reversed = false
        rightParallelEncoder.reversed = true
        perpendicularEncoder.reversed = true
    }

    private var lastLeftParallelReading: Int = 0
    private var lastRightParallelReading: Int = 0
    private var lastPerpendicularReading: Int = 0

    override var poseVel: Pose = Pose()
    override var pose: Pose = Pose()

    override fun init(startPos: Pose) {
        pose = startPos
    }

    val ep = ElapsedTime()
    override fun update() {
        var deltaLeftParallelReading: Double = (leftParallelEncoder.getPosition() - lastLeftParallelReading).toDouble()
        var deltaRightParallelReading: Double = (rightParallelEncoder.getPosition() - lastRightParallelReading).toDouble()
        var deltaPerpendicularReading: Double = (perpendicularEncoder.getPosition() - lastPerpendicularReading).toDouble()

        if (__COIN) {
            log("WheelVelParL", deltaLeftParallelReading / ep.seconds())
            log("WheelVelParR", deltaRightParallelReading / ep.seconds())
            log("WheelVelPerp", deltaPerpendicularReading / ep.seconds())
        }
        lastLeftParallelReading += deltaLeftParallelReading.toInt()
        lastRightParallelReading += deltaRightParallelReading.toInt()
        lastPerpendicularReading += deltaPerpendicularReading.toInt()
        log("LeftPos", lastLeftParallelReading)
        log("RightPos", lastRightParallelReading)
        log("PerpPos", lastPerpendicularReading)

        /*deltaLeftParallelReading /= FORWARD_TICKS_PER_CM
        deltaRightParallelReading /= FORWARD_TICKS_PER_CM
        deltaPerpendicularReading /= LATERAL_TICKS_PER_CM*/
        deltaLeftParallelReading *= WheelsParTicksToCm
        deltaRightParallelReading *= WheelsParTicksToCm
        deltaPerpendicularReading *= WheelsPerpTicksToCm

        val deltaHeading: Double = (deltaRightParallelReading - deltaLeftParallelReading) /
                -abs(LEFT_PARALLEL_OFFSET - RIGHT_PARALLEL_OFFSET)

        deltaLeftParallelReading += deltaHeading * LEFT_PARALLEL_OFFSET
        deltaRightParallelReading += deltaHeading * RIGHT_PARALLEL_OFFSET
        deltaPerpendicularReading += deltaHeading * PERPENDICULAR_OFFSET

        val relativeDeltaForward = (deltaLeftParallelReading + deltaRightParallelReading) / 2
        val relativeDeltaStrafe = deltaPerpendicularReading

        pose += Pose(
                relativeDeltaForward * cos(pose.h + deltaHeading / 2) - relativeDeltaStrafe * sin(pose.h + deltaHeading / 2),
                relativeDeltaForward * sin(pose.h + deltaHeading / 2) + relativeDeltaStrafe * cos(pose.h + deltaHeading / 2),
                deltaHeading
        )
        if (USE_IMU_LOCALIZER) {
            if (USE_COMBINED_LOCALIZER) {
                if (!timmy.localizerAccessed) {
                    timmy.localizerAccessed = true
                    pose.h = angNorm(timmy.yaw)
                }
            } else {
                pose.h = angNorm(timmy.yaw)
            }
        } else {
            //log("timmyhead", angNorm(timmy.yaw))
        }
        //log("poseh", angNorm(pose.h))
        log("CurPos", pose)
    }
}
