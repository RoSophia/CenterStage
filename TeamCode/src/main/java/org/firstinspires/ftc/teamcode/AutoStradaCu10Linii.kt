package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.config.Config
import com.outoftheboxrobotics.photoncore.Photon
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.Auto10Lini.MAX_1_FRAC
import org.firstinspires.ftc.teamcode.Auto10Lini.MAX_STACK_FRAC
import org.firstinspires.ftc.teamcode.Auto10Lini.PeruPose
import org.firstinspires.ftc.teamcode.Auto10Lini.justDraw
import org.firstinspires.ftc.teamcode.Auto10Lini.l1v
import org.firstinspires.ftc.teamcode.Auto10Lini.l2v
import org.firstinspires.ftc.teamcode.Auto10Lini.leftPos
import org.firstinspires.ftc.teamcode.Auto10Lini.lh
import org.firstinspires.ftc.teamcode.Auto10Lini.m1v
import org.firstinspires.ftc.teamcode.Auto10Lini.m2v
import org.firstinspires.ftc.teamcode.Auto10Lini.midPos
import org.firstinspires.ftc.teamcode.Auto10Lini.p1v
import org.firstinspires.ftc.teamcode.Auto10Lini.p2v
import org.firstinspires.ftc.teamcode.Auto10Lini.parkPos
import org.firstinspires.ftc.teamcode.Auto10Lini.ph
import org.firstinspires.ftc.teamcode.Auto10Lini.pmf
import org.firstinspires.ftc.teamcode.Auto10Lini.putPos
import org.firstinspires.ftc.teamcode.Auto10Lini.r1v
import org.firstinspires.ftc.teamcode.Auto10Lini.r2v
import org.firstinspires.ftc.teamcode.Auto10Lini.rh
import org.firstinspires.ftc.teamcode.Auto10Lini.rightPos
import org.firstinspires.ftc.teamcode.Auto10Lini.s1v
import org.firstinspires.ftc.teamcode.Auto10Lini.s2v
import org.firstinspires.ftc.teamcode.Auto10Lini.sh
import org.firstinspires.ftc.teamcode.Auto10Lini.stackPos
import org.firstinspires.ftc.teamcode.Auto10Lini.targetPreload
import org.firstinspires.ftc.teamcode.hardware.Intakes.SDown
import org.firstinspires.ftc.teamcode.hardware.Intakes.SInvert
import org.firstinspires.ftc.teamcode.hardware.Intakes.SNothing
import org.firstinspires.ftc.teamcode.hardware.Intakes.SPStack1
import org.firstinspires.ftc.teamcode.hardware.Intakes.SPStack2
import org.firstinspires.ftc.teamcode.hardware.Intakes.SStack1
import org.firstinspires.ftc.teamcode.hardware.Intakes.SStack2
import org.firstinspires.ftc.teamcode.hardware.ZaPaiplain
import org.firstinspires.ftc.teamcode.pp.Trajectory
import org.firstinspires.ftc.teamcode.utils.Pose
import org.firstinspires.ftc.teamcode.utils.RobotFuncs
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.cam
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.clown
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.controller
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.diffy
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.endma
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.initAuto
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.initma
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.intake
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.localizer
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.log
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.moveSwerve
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.pipeline
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.pp
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.preinit
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.startma
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.telemetry
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.update
import org.firstinspires.ftc.teamcode.utils.RobotVars.AutoInitPos
import org.firstinspires.ftc.teamcode.utils.RobotVars.AutoResult
import org.firstinspires.ftc.teamcode.utils.RobotVars.DiffyDown
import org.firstinspires.ftc.teamcode.utils.RobotVars.DiffyUp
import org.firstinspires.ftc.teamcode.utils.RobotVars.GhearaSINCHIS
import org.firstinspires.ftc.teamcode.utils.RobotVars.GhearaSDESCHIS
import org.firstinspires.ftc.teamcode.utils.RobotVars.IntakePDown
import org.firstinspires.ftc.teamcode.utils.RobotVars.IntakePStack1
import org.firstinspires.ftc.teamcode.utils.RobotVars.IntakePStack2
import org.firstinspires.ftc.teamcode.utils.RobotVars.IntakePower
import org.firstinspires.ftc.teamcode.utils.Vec2d

@Config
object Auto10Lini {
    @JvmField
    var targetPreload = 2

    @JvmField
    var justDraw = false

    @JvmField
    var leftPos = Pose(-82.0, 6.0, 1.57)

    @JvmField
    var lh = Vec2d(0.6, 0.95)

    @JvmField
    var l1v = Vec2d()

    @JvmField
    var l2v = Vec2d()

    @JvmField
    var midPos = Pose(-122.0, -11.0, 0.0)

    @JvmField
    var m1v = Vec2d(50.0, 2.0)

    @JvmField
    var m2v = Vec2d(25.0, 3.0)

    @JvmField
    var rightPos = Pose(-82.0, -13.0, -1.57)

    @JvmField
    var rh = Vec2d(0.6, 0.95)

    @JvmField
    var r1v = Vec2d(0.0, 0.0)

    @JvmField
    var r2v = Vec2d(0.0, 0.0)

    @JvmField
    var stackPos = Pose(-128.0, 55.0, 1.57)

    @JvmField
    var sh = Vec2d(0.6, 0.95)

    @JvmField
    var s1v = Vec2d(0.0, 0.0)

    @JvmField
    var s2v = Vec2d(0.0, 0.0)

    @JvmField
    var putPos = Pose(-54.0, -235.0, 1.57)

    @JvmField
    var ph = Vec2d(0.6, 0.95)

    @JvmField
    var p1v = Vec2d(230.0, 4.6)

    @JvmField
    var p2v = Vec2d(100.0, 2.34)

    @JvmField
    var pmf = 1.2

    @JvmField
    var PeruPose = Pose(0.0, 120.0, 0.0)

    @JvmField
    var parkPos = Pose(-80.0, -235.0, 1.57)

    @JvmField
    var MAX_1_FRAC = 1.2

    @JvmField
    var MAX_STACK_FRAC = 0.7
}

@Photon
//@Autonomous(name = "አውራ ጎዳና ከአስር መንገዶች ጋር")
class AutoStradaCu10Linii : LinearOpMode() {
    private lateinit var peruTraj: Trajectory
    private lateinit var preloadTraj: Trajectory
    private lateinit var stackTraj: Trajectory
    private lateinit var putTraj: Trajectory
    private lateinit var stackTraj2: Trajectory
    private lateinit var putTraj2: Trajectory
    private lateinit var goPark: Trajectory
    fun getTrajectories() {

        peruTraj = Trajectory(AutoInitPos, 0.0, PeruPose)

        return
        preloadTraj = when (targetPreload) {
            0 -> {
                Trajectory(AutoInitPos, 0.0, leftPos, l1v, l2v, lh, MAX_1_FRAC)
            }

            1 -> {
                Trajectory(AutoInitPos, 0.0, midPos, m1v, m2v, Vec2d(), MAX_1_FRAC)
            }

            else -> {
                Trajectory(AutoInitPos, 0.0, rightPos, r1v, r2v, rh, MAX_1_FRAC)
            }
        }
        preloadTraj.addActionE(0.0) { intake.status = SInvert }

        stackTraj = Trajectory(preloadTraj.end, 0.0, stackPos, s1v, s2v, sh, MAX_STACK_FRAC)
        stackTraj.addActionS(0.0) { intake.status = SNothing }
        stackTraj.addActionE(30.0) { intake.status = SPStack1 }
        stackTraj.addActionE(0.0) { intake.status = SStack1 }

        putTraj = Trajectory(stackTraj.end, 0.0, putPos, p1v, p2v, ph, pmf, 30.0, 50.0)
        putTraj.addActionS(0.0) { intake.status = SNothing; clown.position = GhearaSINCHIS }
        putTraj.addActionE(100.0) { diffy.targetPos = DiffyUp }

        stackTraj2 = Trajectory(putTraj.end, 0.0, stackPos, p2v, p1v, ph, pmf, 30.0, 50.0)
        stackTraj2.addActionS(0.0) { clown.position = GhearaSDESCHIS }
        stackTraj2.addActionS(80.0) { clown.position = GhearaSINCHIS; diffy.targetPos = DiffyDown }
        stackTraj2.addActionS(0.0) { intake.status = SNothing }
        stackTraj2.addActionE(30.0) { intake.status = SPStack2 }
        stackTraj2.addActionE(0.0) { intake.status = SStack2 }

        putTraj2 = Trajectory(stackTraj2.end, 0.0, putPos, p1v, p2v, ph, pmf, 30.0, 50.0)
        putTraj2.addActionS(0.0) { intake.status = SNothing; clown.position = GhearaSINCHIS }
        putTraj2.addActionE(100.0) { diffy.targetPos = DiffyUp }
        putTraj2.addActionE(0.0) { clown.position = GhearaSDESCHIS }

        goPark = Trajectory(putTraj2.end, 0.0, parkPos, Vec2d(), Vec2d(), ph, pmf)
        goPark.addActionS(0.0) { clown.position = GhearaSDESCHIS }
        goPark.addActionS(70.0) { diffy.targetPos = DiffyDown }
    }

    var lt = 0
    fun updateTraj() {
        if (lt == 0) {
            if (!pp.busy) {
                pp.startFollowTraj(peruTraj)
                lt = 1
            }
        } else if (lt == 1) {
            if (!pp.busy) {
                requestOpModeStop()
            }
        }
        if (lt == 1) {
            if (!pp.busy) {
                pp.startFollowTraj(stackTraj)
                lt = 2
            }
        } else if (lt == 2) {
            if (!pp.busy) {
                pp.startFollowTraj(putTraj)
                lt = 7
            }
        } else if (lt == 3) {
            if (!pp.busy) {
                pp.startFollowTraj(stackTraj2)
                lt = 4
            }
        } else if (lt == 4) {
            if (!pp.busy) {
                pp.startFollowTraj(putTraj2)
                lt = 5
            }
        } else if (lt == 5) {
            if (!pp.busy) {
                pp.startFollowTraj(stackTraj2)
                lt = 6
            }
        } else if (lt == 6) {
            if (!pp.busy) {
                pp.startFollowTraj(putTraj2)
                lt = 7
            }
        } else if (lt == 7) {
            if (!pp.busy) {
                pp.startFollowTraj(goPark)
                lt = 8
            }
        } else if (lt == 8) {
            if (!pp.busy) {
                requestOpModeStop()
            }
        }
    }

    override fun runOpMode() {
        preinit()
        initma(this)
        initAuto()
        intake.status = SDown
        while (!isStarted) {
            targetPreload = AutoResult
            telemetry.addData("TargetPreload", targetPreload)
            telemetry.update()
            sleep(5)
        }
        waitForStart()
        cam.stop()
        startma()

        getTrajectories()

        while (!isStopRequested) {
            if (justDraw) {
                moveSwerve()
                log("Cpos", localizer.pose)
                getTrajectories()
                pp.drawTraj(preloadTraj)
                pp.drawTraj(stackTraj)
                pp.drawTraj(putTraj)
                pp.drawTraj(stackTraj2)
            } else {
                updateTraj()
                pp.update()
            }
            update()
        }

        endma()
    }

}