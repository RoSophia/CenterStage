package org.firstinspires.ftc.teamcode.pp

import org.firstinspires.ftc.teamcode.utils.Pose

class ThreeWheelLocalizer : Localizer {

    override fun getPose(): Pose {
        return Pose()
    }

    override fun getPoseVel(): Pose {
        return Pose()
    }

    override fun init() {}
}