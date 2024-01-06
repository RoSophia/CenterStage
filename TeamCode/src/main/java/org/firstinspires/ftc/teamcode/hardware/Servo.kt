package org.firstinspires.ftc.teamcode.hardware

import com.outoftheboxrobotics.photoncore.hardware.servo.PhotonServo
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.Servo.Direction
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.hardwareMap
import org.firstinspires.ftc.teamcode.utils.Util.epsEq

class MServo(name: String, reverse: Boolean, initP: Double?) {
    constructor(n: String) : this(n, false, null)
    constructor(n: String, v: Double) : this(n, false, v)
    constructor(n: String, v: Boolean) : this(n, v, null)
    val s: PhotonServo = hardwareMap.get(Servo::class.java, name) as PhotonServo

    init {
        s.direction = if (reverse) Direction.REVERSE else Direction.FORWARD
        if (initP != null) {
            s.position = initP
        }
    }

    var position: Double = initP ?: 20.0
        set(v) {
            if (!epsEq(v, field)) {
                s.position = v
                field = v
            }
        }

    var direction: Direction = if (reverse) Direction.REVERSE else Direction.FORWARD
        set(v) {
            if (field != v) {
                s.direction = v
                field = v
            }
        }
}