package org.firstinspires.ftc.teamcode.hardware

import com.outoftheboxrobotics.photoncore.hardware.motor.PhotonDcMotor
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit
import org.firstinspires.ftc.teamcode.utils.RobotFuncs

class Motor(name: String, encoder: Boolean, rev: Boolean, overdrive: Boolean) {
    private val m: PhotonDcMotor = RobotFuncs.hardwareMap.get(DcMotorEx::class.java, name) as PhotonDcMotor

    var reverse: Boolean = rev
        set(v) {
            field = v
            //m.power = if (field) -power else power
        }

    init {
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

    val currentPosition: Int
        get() {
            return m.currentPosition
        }

    val current: Double
        get() {
            return m.getCurrent(CurrentUnit.MILLIAMPS)
        }

    var power: Double = 0.0
        set(p) {
            if (p != field) {
                field = p
                //m.power = if (reverse) -p else p
            }
        }
}