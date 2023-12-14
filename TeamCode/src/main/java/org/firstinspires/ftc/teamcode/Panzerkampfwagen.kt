package org.firstinspires.ftc.teamcode

import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.log
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.preinit

@TeleOp
@Disabled
class Panzerkampfwagen : OpMode() {

    lateinit var imu: BNO055IMU
    override fun init() {
        preinit()
        imu = hardwareMap.get(BNO055IMU::class.java, "imu")
        val parameters = BNO055IMU.Parameters()
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS
        parameters.mode = BNO055IMU.SensorMode.IMU
        imu.initialize(parameters)
    }

    override fun start() {
    }

    override fun loop() {
        log("Angel", imu.angularOrientation)
    }

    override fun stop() {
        imu.close()
    }

}