package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.hardware.Timmy
import org.firstinspires.ftc.teamcode.utils.RobotFuncs
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.preinit

@TeleOp(name = "我")
@Disabled
class opiatra : OpMode() {
    private lateinit var tim: Timmy
    override fun init() {
        preinit()
        RobotFuncs.hardwareMap = hardwareMap
        tim = Timmy("imu")
        tim.initThread()
    }

    override fun loop() {

    }

    override fun stop() {
        tim.close()
    }

}