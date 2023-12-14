package org.firstinspires.ftc.teamcode.hardware

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.firstinspires.ftc.teamcode.utils.RobotFuncs
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.log

class Motor(name: String, encoder: Boolean, rev: Boolean, overdrive: Boolean) {
    private val m: DcMotorEx

    var reverse: Boolean = rev
        set(v) {
            field = v
            m.power = if (field) -power else power
            reverse
            field = v
        }

    init {
        m = RobotFuncs.hardwareMap.get(DcMotorEx::class.java, name)
        if (overdrive) {
            val mconf = m.motorType.clone()
            mconf.achieveableMaxRPMFraction = 1.0
            m.motorType = mconf
        }
        if (encoder) {
            m.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
            m.mode = DcMotor.RunMode.RUN_USING_ENCODER
        } else {
            m.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        }
        m.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        reverse = rev
        m.power = 0.0
    }

    var power: Double = 0.0
        set(p) {
            if (p != field) {
                field = p
                m.power = if (reverse) -p else p
            }
        }
}