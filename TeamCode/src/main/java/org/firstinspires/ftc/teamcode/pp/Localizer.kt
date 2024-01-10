package org.firstinspires.ftc.teamcode.pp

import org.firstinspires.ftc.teamcode.utils.Pose
import org.firstinspires.ftc.teamcode.utils.Util.angNorm
import org.firstinspires.ftc.teamcode.utils.Util.epsEq
import org.firstinspires.ftc.teamcode.utils.Vec2d
import kotlin.math.cos
import kotlin.math.sin

interface Localizer {
    var pose: Pose
    var poseVel: Pose

    fun init(startPos: Pose)
    fun update()
    //fun start()
    //fun close()

    fun relativeOdometryUpdate(fieldPose: Pose, robotPoseDelta: Pose): Pose {
        val dtheta = robotPoseDelta.h
        val (sineTerm, cosTerm) = if (epsEq(dtheta, 0.0)) {
            1.0 - dtheta * dtheta / 6.0 to dtheta / 2.0
        } else {
            sin(dtheta) / dtheta to (1 - cos(dtheta)) / dtheta
        }

        val fieldPositionDelta = Vec2d(
                sineTerm * robotPoseDelta.x - cosTerm * robotPoseDelta.y,
                cosTerm * robotPoseDelta.x + sineTerm * robotPoseDelta.y
        )

        val fieldPoseDelta = Pose(fieldPositionDelta.rotated(fieldPose.h), robotPoseDelta.h)

        return Pose(
                fieldPose.x + fieldPoseDelta.x,
                fieldPose.y + fieldPoseDelta.y,
                angNorm(fieldPose.h + fieldPoseDelta.h)
        )
    }
}