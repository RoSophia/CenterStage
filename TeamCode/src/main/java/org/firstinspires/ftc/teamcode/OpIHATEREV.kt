package org.firstinspires.ftc.teamcode

import com.outoftheboxrobotics.photoncore.Photon
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.hardware.Intakes.SIntake
import org.firstinspires.ftc.teamcode.hardware.Intakes.SInvert
import org.firstinspires.ftc.teamcode.hardware.Intakes.SNothing
import org.firstinspires.ftc.teamcode.utils.RobotFuncs
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.clown
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.controller
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.diffy
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.endma
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.initma
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.intake
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.moveSwerve
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.preinit
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.slides
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.startma
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.swerve
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.timmy
import org.firstinspires.ftc.teamcode.utils.RobotVars.*
import org.firstinspires.ftc.teamcode.utils.Util.epsEq

@Photon(maximumParallelCommands = 10)
@TeleOp(name = "我討厭修訂")
class OpIHATEREV : LinearOpMode() {
    override fun runOpMode() {
        preinit()
        initma(this)

        waitForStart()

        startma()

        while (!isStopRequested) {
            if (controller.C1A == controller.PRESSED) {
                swerve.locked = true
                swerve.move(0.1, 0.0, 0.0)
            } else {
                swerve.locked = false
            }

            if (controller.C2RB == controller.JUST_PRESSED) {
                if (epsEq(clown.position, GhearaSDESCHIS)) {
                    clown.position = GhearaSINCHIS
                    intake.status = SNothing
                } else {
                    clown.position = GhearaSDESCHIS
                }
            }
            if (controller.C2Y == controller.JUST_PRESSED) {
                if (intake.status == SIntake) {
                    intake.status = SNothing
                } else {
                    intake.status = SIntake
                }
            }
            if (controller.C2A == controller.JUST_PRESSED) {
                diffy.targetPos = DiffyUp
                diffy.targetDiff = DiffyfUp
            }
            if (controller.C2X == controller.JUST_PRESSED) {
                diffy.targetPos = DiffyUp
                diffy.targetDiff = DiffyfDown
            }
            if (controller.C2B == controller.JUST_PRESSED) {
                diffy.targetPos = DiffyDown
                diffy.targetDiff = DiffyfUp
                slides.setTarget(RBOT_POS)
            }
            val g2coef = 1.0 - 0.6 * gamepad2.right_trigger
            if (!epsEq(gamepad2.right_stick_y.toDouble(), 0.0)) {
                slides.power = -gamepad2.right_stick_y.toDouble() * g2coef
            }
            if (controller.C2DU == controller.JUST_PRESSED) {
                slides.setTarget(RTOP_POS)
            }
            if (controller.C2DR == controller.JUST_PRESSED) {
                slides.setTarget(RMID_POS)
            }
            if (controller.C2LT == controller.JUST_PRESSED) {
                timmy.openAvion()
            }
            if (controller.C2DL == controller.JUST_PRESSED) {
                intake.status = SInvert
            }

            if (__STATUS != 20) {
                intake.status = __STATUS
                __STATUS = 20
            }
            moveSwerve()
            RobotFuncs.update()
        }

        endma()
    }
}
