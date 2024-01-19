package org.firstinspires.ftc.teamcode.hardware

import org.firstinspires.ftc.teamcode.hardware.Intakes.SGoDownForOuttake
import org.firstinspires.ftc.teamcode.hardware.Intakes.SIntake
import org.firstinspires.ftc.teamcode.hardware.Intakes.SNothing
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.clown
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.intake
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.log
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.logs
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.slides
import org.firstinspires.ftc.teamcode.utils.RobotVars.DIFGRAT
import org.firstinspires.ftc.teamcode.utils.RobotVars.DIFLOFF
import org.firstinspires.ftc.teamcode.utils.RobotVars.DIFROFF
import org.firstinspires.ftc.teamcode.utils.RobotVars.DiffyIntakeDownPoint
import org.firstinspires.ftc.teamcode.utils.RobotVars.DiffyIntakeDownPoint2
import org.firstinspires.ftc.teamcode.utils.RobotVars.DiffyIntakeUp
import org.firstinspires.ftc.teamcode.utils.RobotVars.DiffyPid
import org.firstinspires.ftc.teamcode.utils.RobotVars.GhearaSDESCHIS
import org.firstinspires.ftc.teamcode.utils.RobotVars.GhearaSINCHIS
import org.firstinspires.ftc.teamcode.utils.RobotVars.USE_DIFFY
import org.firstinspires.ftc.teamcode.utils.RobotVars.nrRots
import org.firstinspires.ftc.teamcode.utils.Util.epsEq

class Diffy(name: String) {
    init {
        nrRots.remove(name + "LE")
        //nrRots[name + "LE"] = -1
        nrRots.remove(name + "RE")
        //nrRots[name + "RE"] = -1
    }

    val LS = CServo(name + "L", DIFLOFF, DIFGRAT, false, DiffyPid)
    val RS = CServo(name + "R", DIFROFF, DIFGRAT, false, DiffyPid)

    private var lip = 0
    private var lst = 0

    val cangle: Double
        get() = (LS.e.angle + RS.e.angle) / 2
    val cpos: Double
        get() = (RS.e.angle - LS.e.angle) / 2

    fun update() {
        if (USE_DIFFY) {
            logs("DiffyLSP", LS.e.angle)
            logs("DiffyRSP", RS.e.angle)
            logs("DiffyCPOS", cpos)
            logs("DiffyCTurn", cangle)
            if ((targetPos > DiffyIntakeDownPoint && cpos < DiffyIntakeDownPoint) || (targetPos < DiffyIntakeDownPoint && cpos > DiffyIntakeDownPoint && cpos < DiffyIntakeDownPoint2)) {
                if (intake.status != SGoDownForOuttake) {
                    lip = intake.status
                }
                intake.status = SGoDownForOuttake
            } else {
                if (lip != SGoDownForOuttake) {
                    if (lip == SIntake && targetPos > 0.5) {
                        intake.status = SNothing
                    } else {
                        intake.status = lip
                    }
                    lip = SGoDownForOuttake
                }
            }
            RS.update()
            LS.update()
        }
    }

    fun init() {
        if (USE_DIFFY) {
            LS.pt = 0.0
            RS.pt = 0.0
            LS.init()
            RS.init()
        }
    }

    fun close() {
        if (USE_DIFFY) {
            LS.close()
            RS.close()
        }
    }

    /// Since the left servo is not flipped the PIDF coefficients have to be negative,
    /// and also the angle and Diff and Pos terms be reversed
    fun updateTarget() {
        LS.pt = targetDiff - targetPos / 2.0
        RS.pt = targetDiff + targetPos / 2.0
    }

    var targetDiff = 0.0
        set(v) {
            if (v != field) {
                field = v
                updateTarget()
            }
        }

    var lcp = -100.0
    var targetPos = 0.0
        set(v) {
            if (v != field) {
                if (slides.pos < 0.5 && targetPos > 0.5) {
                    clown.position = GhearaSINCHIS
                }
                if (slides.pos > 0.5 && targetPos < 0.5 && epsEq(clown.position, GhearaSDESCHIS)) {
                    lcp = GhearaSDESCHIS
                    log("DiffyUpClose", slides.pos)
                    clown.position = GhearaSINCHIS
                }
                if (epsEq(lcp, GhearaSDESCHIS) && slides.pos < 0.5) {
                    clown.position = lcp
                    log("DiffyDownClose", slides.pos)
                    lcp = -100.0
                }
                field = v
                updateTarget()
            }
        }
}