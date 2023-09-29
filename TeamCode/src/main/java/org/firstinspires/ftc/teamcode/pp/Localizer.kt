package org.firstinspires.ftc.teamcode.pp

import org.firstinspires.ftc.teamcode.utils.Pose

interface Localizer {
    fun getPose(): Pose
    fun getPoseVel(): Pose
    fun init()
}