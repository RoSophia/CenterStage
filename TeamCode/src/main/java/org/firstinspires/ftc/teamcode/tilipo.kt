package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.Gamepad.LedEffect
import org.firstinspires.ftc.teamcode.hardware.Intakes
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.avion
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.clown
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.controller
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.create_god
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.drawRobot
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.endma
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.initma
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.intake
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.log
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.moveSwerve
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.preinit
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.slides
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.startma
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.swerve
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.update
import org.firstinspires.ftc.teamcode.utils.RobotVars.*
import org.firstinspires.ftc.teamcode.utils.Util

object tilipo {
    fun runOpMode(lom: LinearOpMode, fn: Boolean) { /// TODO: dashboard.sendImage(NekoArc)
        TimmyToClose = false
        preinit()
        __AutoShort = OpModeKMSShort
        initma(lom, OpModeKMS)
        create_god()

        lom.waitForStart()

        startma()
        //lom.gamepad2.ledQueue.add(st)
        var curstackp = 6

        while (!lom.isStopRequested) {
            if (controller.C1A == controller.PRESSED) {
                swerve.locked = true
                swerve.move(0.1, 0.0, 0.0)
            } else {
                swerve.locked = false
            }

            if (controller.C1RB == controller.JUST_PRESSED) {
                intake.status = when (curstackp) {
                    6 -> Intakes.SStack6
                    5 -> Intakes.SStack5
                    4 -> Intakes.SStack4
                    3 -> Intakes.SStack3
                    2 -> Intakes.SStack2
                    else -> Intakes.SStack1
                }
                --curstackp
                if (curstackp == 0) {
                    curstackp = 6
                }
            }

            if (controller.C1Y == controller.JUST_PRESSED || controller.C2Y == controller.JUST_PRESSED) {
                curstackp = 6
                if (intake.running) {
                    intake.status = Intakes.SNothing
                } else {
                    intake.status = Intakes.SIntake
                }
            }

            if (controller.C2B == controller.JUST_PRESSED || controller.C1B == controller.JUST_PRESSED) {
                clown.goDown()
            }

            if (controller.C2X == controller.JUST_PRESSED) {
                if (intake.status == Intakes.SFinalHang) {
                    intake.status = Intakes.SNothing
                } else {
                    intake.status = Intakes.SFinalHang
                }
            }
            /*
            if (controller.C1LB == controller.JUST_PRESSED) {
                TrajectorySequence()
                        .aa { intake.status = Intakes.SStack2 }
                        .sl(INTAKEWAIT2)
                        .aa { intake.status = Intakes.SStack3 }
                        .runAsync()
            }
            if (controller.C1X == controller.JUST_PRESSED) {
                intake.intake.power = IntakeRevPower
            }
            if (controller.C1DU == controller.JUST_PRESSED) {
                clown.catchPixel()
            }
            if (controller.C1DD == controller.JUST_PRESSED) {
                clown.goUp(0)
            }
            if (controller.C1B == controller.JUST_PRESSED) {
                clown.goDown()
            }
            if (controller.C1RSB == controller.JUST_PRESSED) {
                clown.goPreloadUp()
            }
            if (controller.C1LSB == controller.JUST_PRESSED) {
                clown.goUp(2)
            }*/

            if (controller.C2PS == controller.JUST_PRESSED) {
                if (!slides.RIDICAREEEEEEEEEE) {
                    slides.youShouldHangYourselfNOW()
                } else {
                    slides.RIDICAREEEEEEEEEE = false
                }
            }

            if (controller.C2LB == controller.JUST_PRESSED) {
                clown.ghearaFar?.position = ClownFDeschis
                val st = LedEffect.Builder().addStep(1.0, 0.0, 1.0, 200).addStep(0.0, 1.0, 1.0, 200).setRepeating(true).build()
                lom.gamepad2.runLedEffect(st)
            }

            if (controller.C2RB == controller.JUST_PRESSED) {
                clown.ghearaNear?.position = ClownNDeschis
            }

            if (controller.C2DL == controller.JUST_PRESSED) {
                clown.goLeft()
            }
            if (controller.C2DR == controller.JUST_PRESSED) {
                clown.goRight()
            }
            if (controller.C2A == controller.JUST_PRESSED || controller.C1DD == controller.JUST_PRESSED) {
                curstackp = 6
                clown.goUp(0)
            }
            if (controller.C1DU == controller.JUST_PRESSED) {
                curstackp = 6
                clown.goUpp()
            }
            if (controller.C2RT == controller.JUST_PRESSED) {
                clown.close()
            }

            val g2coef = 1.0 /*- 0.6 * lom.gamepad2.right_trigger*/
            if (!Util.epsEq(lom.gamepad2.right_stick_y.toDouble(), 0.0)) {
                slides.power = -lom.gamepad2.right_stick_y.toDouble() * g2coef
            }
            if (controller.C2LT == controller.JUST_PRESSED) {
                avion.position = AvionDeschis
            }
            if (controller.C2LSB == controller.JUST_PRESSED) {
                avion.position = AvionInchis
            }
            if (controller.C2DU == controller.JUST_PRESSED) {
                clown.catchPixel()
            }
            if (controller.C2DD == controller.JUST_PRESSED) {
                intake.status = Intakes.SInvert
            }

            if (__IntakeSetStatus != 20) {
                intake.status = __IntakeSetStatus
                __IntakeSetStatus = 20
            }

            if (__LOG_STATUS) {
                log("DiffyActualPos", clown.actualDiffyPos)
            }

            drawRobot()

            moveSwerve(fn)
            update()
        }

        TimmyToClose = true
        endma()
    }
}