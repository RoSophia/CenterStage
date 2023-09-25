package org.firstinspires.ftc.teamcode.hardware

import org.firstinspires.ftc.teamcode.hardware.CServo
import org.firstinspires.ftc.teamcode.hardware.Motor

class Swerve {
    private val lf = SwerveModule("LF")
    private val lb = SwerveModule("LB")
    private val rf = SwerveModule("RF")
    private val rb = SwerveModule("RB")

    fun close() {
        lf.close()
        lb.close()
        rf.close()
        rb.close()
    }

}