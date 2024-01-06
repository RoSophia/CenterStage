package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.outoftheboxrobotics.photoncore.Photon
import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.PKW.PAUER
import org.firstinspires.ftc.teamcode.PKW.PAUER2
import org.firstinspires.ftc.teamcode.PKW.inv
import org.firstinspires.ftc.teamcode.PKW.inv2
import org.firstinspires.ftc.teamcode.PKW.string
import org.firstinspires.ftc.teamcode.PKW.string2
import org.firstinspires.ftc.teamcode.PKW.type
import org.firstinspires.ftc.teamcode.hardware.AbsEnc
import org.firstinspires.ftc.teamcode.hardware.MServo
import org.firstinspires.ftc.teamcode.hardware.Motor
import org.firstinspires.ftc.teamcode.utils.RobotFuncs
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.hardwareMap
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.log
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.preinit

@Config
object PKW {
    @JvmField
    var type: Int = 1
    @JvmField
    var string: String = ""
    @JvmField
    var PAUER: Double = 0.0
    @JvmField
    var string2: String = ""
    @JvmField
    var PAUER2: Double = 0.0
    @JvmField
    var inv: Boolean = false
    @JvmField
    var inv2: Boolean = false
}

@Photon
@TeleOp
class Panzerkampfwagen : OpMode() {
    lateinit var imu: BNO055IMU
    lateinit var crs: CRServo
    lateinit var s: MServo
    lateinit var s2: MServo
    lateinit var abe: AbsEnc
    lateinit var mot: Motor

    override fun init() {
        preinit()
        RobotFuncs.hardwareMap = hardwareMap
        when (type) {
            1 -> {
                crs = hardwareMap.get(CRServo::class.java, string)
            }
            2 -> {
                abe = AbsEnc(string, 0.0)
            }
            3 -> {
                mot = Motor(string, false, false, false)
            }
            4 -> {
                mot = Motor(string, true, false, false)
            }
            5 -> {
                s = MServo(string, inv)
            }
            6 -> {
                s = MServo(string, inv)
                s2 = MServo(string2, inv2)
            }
        }
    }

    override fun start() {
    }

    override fun loop() {
        when (type) {
            1 -> {
                crs.power = PAUER
            }
            2 -> {
                log("pos", abe.pos)
            }
            3 -> {
                mot.power = PAUER
            }
            4 -> {
                mot.power = PAUER
                val tp = TelemetryPacket()
                tp.put("POS", mot.currentPosition)
                FtcDashboard.getInstance().sendTelemetryPacket(tp)
            }
            5 -> {
                s.position = PAUER
            }
            6 -> {
                s.position = PAUER
                s2.position = PAUER2
            }
        }
    }

    override fun stop() {
    }

}