package org.firstinspires.ftc.teamcode.hardware

import com.outoftheboxrobotics.photoncore.hardware.motor.PhotonDcMotor
import com.outoftheboxrobotics.photoncore.hardware.servo.PhotonCRServo
import com.outoftheboxrobotics.photoncore.hardware.servo.PhotonServo
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction
import org.firstinspires.ftc.teamcode.utils.RobotFuncs

class MCRServo(name: String) {
    val s: PhotonCRServo = RobotFuncs.hardwareMap.get(CRServo::class.java, name) as PhotonCRServo
    //val s: CRServo = RobotFuncs.hardwareMap.get(CRServo::class.java, name) //as PhotonCRServo

    var direction: Direction = Direction.FORWARD
        set (v) {
            if (v != field) {
                s.direction = v
                field = v
            }
        }

    var power: Double = 0.0
        set(v) {
            if (v != field) {
                //s.power = v
                field = v
            }
        }
}