package org.firstinspires.ftc.teamcode.pp

import org.firstinspires.ftc.teamcode.utils.Pose

interface Localizer {
    var pose: Pose
    var poseVel: Pose

    fun init()
}