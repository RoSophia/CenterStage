package org.firstinspires.ftc.teamcode.pp

import org.firstinspires.ftc.teamcode.hardware.Swerve

class PurePursuit(private val swerve: Swerve, private val localizer: Localizer) {
    var ctraj = Trajectory()

    fun followTrajAsync(t: Trajectory) {
        ctraj = t
    }

    fun update() {
        val cp = localizer.getPose()
        val cv = localizer.getPoseVel()

        val lk = lookahead()


    }
}