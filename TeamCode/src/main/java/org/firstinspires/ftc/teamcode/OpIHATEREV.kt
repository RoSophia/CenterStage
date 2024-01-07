package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.config.Config
import com.outoftheboxrobotics.photoncore.Photon
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.utils.RobotFuncs
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.clown
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.controller
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.diffy
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.endma
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.funkyL
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.funkyR
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.initma
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.intake
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.log_state
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.moveSwerve
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.preinit
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.ridIntake
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.send_log
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.startma
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.swerve
import org.firstinspires.ftc.teamcode.utils.RobotVars.DIFDOWN
import org.firstinspires.ftc.teamcode.utils.RobotVars.DIFFDOWN
import org.firstinspires.ftc.teamcode.utils.RobotVars.DIFFUP
import org.firstinspires.ftc.teamcode.utils.RobotVars.DIFUP
import org.firstinspires.ftc.teamcode.utils.RobotVars.FUNKYLD
import org.firstinspires.ftc.teamcode.utils.RobotVars.FUNKYLU
import org.firstinspires.ftc.teamcode.utils.RobotVars.FUNKYRD
import org.firstinspires.ftc.teamcode.utils.RobotVars.FUNKYRU
import org.firstinspires.ftc.teamcode.utils.RobotVars.GearaSINCHIS
import org.firstinspires.ftc.teamcode.utils.RobotVars.GhearaSDESCHIS
import org.firstinspires.ftc.teamcode.utils.RobotVars.IntakePDown
import org.firstinspires.ftc.teamcode.utils.RobotVars.IntakePUp
import org.firstinspires.ftc.teamcode.utils.RobotVars.IntakePower
import org.firstinspires.ftc.teamcode.utils.Util.epsEq

@Config
object IHRP {
    @JvmField
    var AP = 1.0

    @JvmField
    var AI = 0.0

    @JvmField
    var AD = 0.0

    @JvmField
    var AF = 0.0

    @JvmField
    var KILL = 1.0

    @JvmField
    var OFFSET = 0.0
}

@Photon(maximumParallelCommands = 10)
@TeleOp(name = "我討厭修訂")
class OpIHATEREV : LinearOpMode() {
    override fun runOpMode() {
        preinit()
        initma(this)

        waitForStart()

        startma()

        while (!isStopRequested) {
            if (controller.C1A == controller.JUST_PRESSED) {
                swerve.locked = !swerve.locked
                swerve.move(0.1, 0.0, 0.0)
            }
            if (controller.C2RB == controller.JUST_PRESSED) {
                if (epsEq(clown.position, GhearaSDESCHIS)) {
                    clown.position = GearaSINCHIS
                } else {
                    clown.position = GhearaSDESCHIS
                }
            }
            if (controller.C2Y == controller.JUST_PRESSED) {
                if (epsEq(ridIntake.position, IntakePUp)) {
                    intake.power = IntakePower
                    ridIntake.position = IntakePDown
                } else {
                    intake.power = 0.0
                    ridIntake.position = IntakePUp
                }
            }
            if (controller.C2DU == controller.JUST_PRESSED) {
                if (epsEq(funkyL.position, FUNKYLD)) {
                    funkyL.position = FUNKYLU
                    funkyR.position = FUNKYRU
                } else {
                    funkyL.position = FUNKYLD
                    funkyR.position = FUNKYRD
                }
            }
            if (controller.C2A == controller.JUST_PRESSED) {
                diffy.targetPos = DIFUP
                diffy.targetDiff = DIFFUP
            }
            if (controller.C2X == controller.JUST_PRESSED) {
                diffy.targetPos = DIFUP
                diffy.targetDiff = DIFFDOWN
            }
            if (controller.C2B == controller.JUST_PRESSED) {
                diffy.targetPos = DIFDOWN
                diffy.targetDiff = DIFFUP
            }

            moveSwerve()
            RobotFuncs.update()
        }

        endma()
    }
}
