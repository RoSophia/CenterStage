package org.firstinspires.ftc.teamcode.auto

import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.auto.AutoVars.GOUPDIST
import org.firstinspires.ftc.teamcode.auto.AutoVars.INTAKEWAIT3
import org.firstinspires.ftc.teamcode.auto.AutoVars.KMS
import org.firstinspires.ftc.teamcode.auto.AutoVars.STACKCORRTIME1
import org.firstinspires.ftc.teamcode.auto.AutoVars.STACKCORRTIME2
import org.firstinspires.ftc.teamcode.auto.AutoVars.WaitPreload
import org.firstinspires.ftc.teamcode.auto.AutoVars.WaitPut
import org.firstinspires.ftc.teamcode.auto.AutoVars.WaitStack1
import org.firstinspires.ftc.teamcode.auto.AutoVars.WaitStack2
import org.firstinspires.ftc.teamcode.auto.AutoVars.WaitStack2Min
import org.firstinspires.ftc.teamcode.auto.AutoVars.WaitStack3
import org.firstinspires.ftc.teamcode.auto.AutoVars.failsafe1
import org.firstinspires.ftc.teamcode.auto.AutoVars.failsafe2
import org.firstinspires.ftc.teamcode.hardware.CameraControls.AutoRed
import org.firstinspires.ftc.teamcode.hardware.Intakes.SIntake
import org.firstinspires.ftc.teamcode.hardware.Intakes.SInvert
import org.firstinspires.ftc.teamcode.hardware.Intakes.SKeep
import org.firstinspires.ftc.teamcode.hardware.Intakes.SStack1
import org.firstinspires.ftc.teamcode.hardware.Intakes.SStack2
import org.firstinspires.ftc.teamcode.hardware.Intakes.SStack3
import org.firstinspires.ftc.teamcode.hardware.Intakes.SStack4
import org.firstinspires.ftc.teamcode.hardware.Intakes.SStack5
import org.firstinspires.ftc.teamcode.hardware.Intakes.SStack6
import org.firstinspires.ftc.teamcode.hardware.Intakes.SUp
import org.firstinspires.ftc.teamcode.hardware.Intakes.SUpulLuiCostacu
import org.firstinspires.ftc.teamcode.utils.Pose
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.clown
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.etime
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.intake
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.log
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.pp
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.slides
import org.firstinspires.ftc.teamcode.utils.RobotVars.ClownFDeschis
import org.firstinspires.ftc.teamcode.utils.RobotVars.DiffyAUp
import org.firstinspires.ftc.teamcode.utils.RobotVars.DiffyUp
import org.firstinspires.ftc.teamcode.utils.RobotVars.DiffyUpSafe
import org.firstinspires.ftc.teamcode.utils.RobotVars.GelenkCenter
import org.firstinspires.ftc.teamcode.utils.RobotVars.GelenkDif
import org.firstinspires.ftc.teamcode.utils.RobotVars.RBOT_POS
import org.firstinspires.ftc.teamcode.utils.RobotVars.RMID_POS
import org.firstinspires.ftc.teamcode.utils.RobotVars.__UPDATE_SENSORS
import org.firstinspires.ftc.teamcode.utils.TrajectorySequence

object Cele10Traiectorii {

    @JvmStatic
    fun getCycleTrajLongNew(ncycle: Int, randomCase: Int, v: LongVals): TrajectorySequence {
        val ts = TrajectorySequence()
                .aa { intake.status = SKeep }
                .aa { clown.targetPos = DiffyUpSafe }
                .at(v.aStartPreload[randomCase].st(0.6).t /// Sets timeout to 0.6
                        .addActionT(0.1) { clown.goPreloadUp() })
                .aa { clown.ghearaFar?.position = ClownFDeschis }
                .sl(WaitPreload)
                .aa { clown.goPreloadDown(); intake.status = SStack6; __UPDATE_SENSORS = true }

        ts.at(v.bPreloadStack[randomCase].s(ts).t)
                .aa { intake.status = SStack5 }
                .slc(WaitStack2, { clown.sensorReadout() == 3 }, WaitStack2Min)
                .failsafeMove({ intake.status = SStack4 }, { clown.sensorReadout() == 3 }, 2, 5, failsafe1, failsafe2)

        ts.aa { clown.catchPixel(); __UPDATE_SENSORS = false }
                .at(v.bStackBackdrop[0].s(ts).cb().t
                        .addActionT(0.6) { intake.status = SInvert }
                        .addActionE(25.0) { intake.status = SUpulLuiCostacu })
        ts.at(v.bStackBackdrop[1].s(ts).sx(v.cBackdropPosX[randomCase]).ce().t
                .addActionS(0.0) { clown.goUp((if (randomCase == 0) -2 else 2) * if (AutoRed) -1 else 1) })
                .aa { clown.open() }
                .sl(WaitPut)

        for (i in 0 until 2) {
            ts.gt { if (etime.seconds() < 25.0) gi(i, 1) else 2 }
            ts.st(gi(i, 1))
            ts.at(v.cBackdropStack[0].s(ts).so(v.stackOffset * i).cb().t
                    .addActionE(0.0) { clown.goDown(); slides.setTarget(RBOT_POS) })
            ts.at(v.cBackdropStack[1].s(ts).so(v.stackOffset * i).ce().st(0.6).t
                    .addActionE(100.0) { intake.status = SStack6 })
                    .aa {
                        when (i) {
                            0 -> intake.status = SStack4
                            1 -> intake.status = SStack2
                            else -> intake.status = SIntake
                        }
                    }
                    .sl(WaitStack1)
                    .aa {
                        when (i) {
                            0 -> intake.status = SStack3
                            1 -> intake.status = SStack1
                            else -> intake.status = SIntake
                        }
                        __UPDATE_SENSORS = true
                    }
                    .slc(WaitStack2, { clown.sensorReadout() == 3 }, WaitStack2Min)
                    .failsafeMove(if (i == 0) ({ intake.status = SStack2 }) else ({ intake.status = SIntake }), { clown.sensorReadout() == 3 }, gi(i, 2), gi(i, 5), failsafe1, failsafe2)
            ts.aa { __UPDATE_SENSORS = false }
                    .aa { clown.catchPixel() }
                    .at(v.bStackBackdrop[0].s(ts).so(v.cBackdropOffset * i - Pose(10.0, 0.0, 0.0)).cb().t
                            .addActionT(0.6) { intake.status = SInvert }
                            //.addActionT(INTAKEWAIT3) { intake.status = SUp }
                            .addActionE(10.0) { intake.status = SUpulLuiCostacu })
            ts.at(v.bStackBackdrop[1].s(ts).so(v.cBackdropOffset * i).ce().t
                    .addActionS(0.0) { clown.goUp(-2) }
                    .addActionE(20.0) { slides.setTarget(RMID_POS) })
                    .aa { clown.open() }
                    .sl(WaitPut)
        }
        ts.st(2)
        ts.at(v.zBackdropPark.ss(v.bStackBackdrop[1].ep).t
                .addActionS(0.0) { clown.open() }
                .addActionS(70.0) { clown.goDown(); slides.setTarget(RBOT_POS) })
                .aa { clown.gelenk?.position = GelenkCenter }
        return ts
    }

    @JvmStatic
    fun getCycleTrajLong(ncycle: Int, randomCase: Int, v: LongVals): TrajectorySequence {
        val ts = TrajectorySequence()
                .aa { intake.status = SKeep }
                .aa { clown.targetPos = DiffyUpSafe }
                .at(v.aStartPreload[randomCase].st(0.6).t /// Sets timeout to 0.6
                        .addActionT(0.1) { clown.goPreloadUp() })
                .aa { clown.ghearaFar?.position = ClownFDeschis }
                .sl(WaitPreload)
                .aa { clown.goPreloadDown(); intake.status = SStack6; __UPDATE_SENSORS = true }

        ts.at(v.bPreloadStack[randomCase].s(ts).t)
                .aa { intake.status = SStack5 }
                .slc(WaitStack2, { clown.sensorReadout() == 3 }, WaitStack2Min)
                .failsafeMove({ intake.status = SStack4 }, { clown.sensorReadout() == 3 }, 2, 5, failsafe1, failsafe2)

        ts.aa { clown.catchPixel(); __UPDATE_SENSORS = false }
                .at(v.bStackBackdrop[0].s(ts).cb().t
                        .addActionT(0.6) { intake.status = SInvert }
                        .addActionE(25.0) { intake.status = SUpulLuiCostacu })
        ts.at(v.bStackBackdrop[1].s(ts).sx(v.cBackdropPosX[randomCase]).ce().t
                .addActionS(0.0) { clown.goUp((if (randomCase == 0) -2 else 2) * if (AutoRed) -1 else 1) })
                .aa { clown.open() }
                .sl(WaitPut)

        for (i in 0 until 3) {
            ts.gt { if (etime.seconds() < 23.2) gi(i, 1) else 1000 }
            ts.st(gi(i, 1))
            ts.aa { log("Going for $i at", etime.seconds())}
            if (i > 1) {
                ts.at(v.xBackdropStack[0].s(ts).so(v.stackOffset * i).cb().t
                        .addActionE(0.0) { clown.goDown(); slides.setTarget(RBOT_POS) })
                ts.at(v.xBackdropStack[1].s(ts).so(v.stackOffset * i).cc().t
                        .addActionE(100.0) { intake.status = SStack6 })
                ts.at(v.xBackdropStack[2].s(ts).so(v.stackOffset * i).ce().st(0.4).t)
                        .aa { intake.status = SStack5 }
                        .sl(WaitStack1 / 2.0)
                        .aa { intake.status = SStack4; }
                        .sl(WaitStack2Min)
                        .aa { clown.catchPixel() }
                ts.at(v.xStackBackdrop[0].s(ts).so(v.cBackdropOffset * i - Pose(10.0, 0.0, 0.0)).cb().t
                        .addActionT(0.6) { intake.status = SInvert })
                ts.at(v.xStackBackdrop[1].s(ts).so(v.cBackdropOffset * i).cc().t
                        .addActionE(10.0) { intake.status = SUpulLuiCostacu })
                ts.at(v.xStackBackdrop[2].s(ts).so(v.cBackdropOffset * i).ce().t
                        .addActionS(0.0) { clown.goUp(-2) }
                        .addActionE(20.0) { slides.setTarget(RMID_POS) })
                        .aa { clown.open() }
                        .sl(WaitPut)
                        .aa {clown.goDown(); slides.setTarget(RBOT_POS); clown.gelenk?.position = GelenkCenter}
                        .sl(100.0)
                ts.st(1002)
            } else {
                ts.at(v.cBackdropStack[0].s(ts).so(v.stackOffset * i).cb().t
                        .addActionE(0.0) { clown.goDown(); slides.setTarget(RBOT_POS) })
                ts.at(v.cBackdropStack[1].s(ts).so(v.stackOffset * i).ce().st(0.6).t
                        .addActionE(100.0) { intake.status = SStack6 })
                        .aa {
                            when (i) {
                                0 -> intake.status = SStack4
                                1 -> intake.status = SStack2
                                else -> intake.status = SIntake
                            }
                        }
                        .sl(WaitStack1)
                        .aa {
                            when (i) {
                                0 -> intake.status = SStack3
                                1 -> intake.status = SStack1
                                else -> intake.status = SIntake
                            }
                            __UPDATE_SENSORS = true
                        }
                        .slc(WaitStack2, { clown.sensorReadout() == 3 }, WaitStack2Min)
                        .failsafeMove(if (i == 0) ({ intake.status = SStack2 }) else ({ intake.status = SIntake }), { clown.sensorReadout() == 3 }, gi(i, 2), gi(i, 5), failsafe1, failsafe2)
                ts.aa { __UPDATE_SENSORS = false }
                        .aa { clown.catchPixel() }
                        .at(v.bStackBackdrop[0].s(ts).so(v.cBackdropOffset * i - Pose(10.0, 0.0, 0.0)).cb().t
                                .addActionT(0.6) { intake.status = SInvert }
                                //.addActionT(INTAKEWAIT3) { intake.status = SUp }
                                .addActionE(10.0) { intake.status = SUpulLuiCostacu })
                ts.at(v.bStackBackdrop[1].s(ts).so(v.cBackdropOffset * i).ce().t
                        .addActionS(0.0) { clown.goUp(-2) }
                        .addActionE(20.0) { slides.setTarget(RMID_POS) })
                        .aa { clown.open() }
                        .sl(WaitPut)
            }
        }

        ts.st(1000)
        ts.aa { log("Parking at", etime.seconds())}
        ts.aa { log("Parking at1", v.zBackdropPark.ss(v.bStackBackdrop[1].ep).sp)}
        ts.aa { log("Parking at2", v.zBackdropPark.ep)}
        ts.at(v.zBackdropPark.ss(v.bStackBackdrop[1].ep).t
                .addActionT(0.0) { log("Currently parkingh", pp.ctraj)}
                .addActionS(0.0) { clown.open() }
                .addActionS(70.0) { clown.goDown(); slides.setTarget(RBOT_POS) })
                .aa { clown.gelenk?.position = GelenkCenter }
        ts.st(1002)
        return ts
    }

    fun gi(i: Int, p: Int) = (i + 1) * 20 + p

    @JvmStatic
    fun getCycleTrajShort(ncycle: Int, randomCase: Int, v: ShortVals): TrajectorySequence {
        val ts = TrajectorySequence()
                .aa { clown.targetPos = DiffyUpSafe }
                .at(v.aStartPreload[randomCase].t
                        .addActionT(0.1) { clown.goPreloadUp() }
                        .addActionS(60.0) { intake.status = SUpulLuiCostacu }) /// Go from start to put preload
                .aa { clown.ghearaFar?.position = ClownFDeschis }
                .aa { clown.targetPos = DiffyUpSafe }
                .sl(0.1)
        ts.at(v.aPreloadBackdrop.s(ts).sx(v.backdropPosX[randomCase]).t /// Set sp to last ep and set ep x
                .addActionE(60.0) {
                    clown.targetPos = DiffyUp
                    clown.targetAngle = DiffyAUp
                    clown.curState = 0
                    clown.gelenk?.position = GelenkCenter + (if (randomCase == 2) -2 else 2) * (if (AutoRed) -1 else 1) * GelenkDif
                }) /// Go from put preload to backboard
                .aa { clown.open() }
                .sl(0.1)

        for (i in 0 until ncycle) {
            /// Put -> Inter1 (Diffy down) -> Inter2 -> Stack (Diffy down + gheara inchisa -> gheara deschisa + intake)
            ts.at(v.backdropStack[0].s(ts).so(v.bStackOffset * i).cb().t /// Set sp; Add offset to ep; Set "Continue begin"; Get traj
                    .addActionS(50.0) { clown.goDown(); slides.setTarget(RBOT_POS) })
            ts.at(v.backdropStack[1].s(ts).so(v.bStackOffset * i).cc().t) /// Set sp (with offset from last ep carried over) ; Add offset to ep; Set "Continue Continue"; Get traj
                    .aa { intake.status = SIntake; }
            ts.at(v.backdropStack[2].s(ts).so(v.bStackOffset * i
                    + Pose(0.0, if (i == 0) (if (AutoRed) -KMS else KMS) else 0.0, 0.0))
                    .ce().t
                    .addActionE(80.0) { intake.status = SStack6 }
            ) /// Set sp (with offset from last ep carried over) ; Add offset to ep; Set "Continue Continue"; Get traj
                    .aa {
                        when (i) {
                            0 -> intake.status = SStack5
                            1 -> intake.status = SStack3
                            else -> intake.status = SIntake
                        }
                    }
                    .sl(WaitStack1)
                    .aa {
                        when (i) {
                            0 -> intake.status = SStack4
                            1 -> intake.status = SStack2
                            else -> intake.status = SIntake
                        }
                    }
                    .slc(WaitStack2, if (i == 2) ({ clown.sensorReadout() > 0 }) else ({ clown.sensorReadout() == 3 }), WaitStack2Min)
                    .failsafeMove(if (i == 0) ({ intake.status = SStack3 }) else if (i == 1) ({ intake.status = SStack2 }) else ({ intake.status = SIntake }),
                            if (i == 2) ({ clown.sensorReadout() > 0 }) else ({ clown.sensorReadout() == 3 }), gi(i, 2), gi(i, 5), failsafe1, failsafe2)
                    .aa { clown.catchPixel() }

            /// Stack -> Inter2 -> Inter1 -> Put (Gheara inchisa -> Diffy up + gheara deschisa)
            ts.at(v.cStackBackdrop[0].s(ts).so(v.dBackdropOffset * i).cb().t
                    .addActionT(INTAKEWAIT3) { intake.status = SUp })
                    .aa { intake.status = SIntake; clown.open() }
            ts.at(v.cStackBackdrop[1].s(ts).so(v.dBackdropOffset * i).cc().t
                    .addActionS(20.0) { intake.status = SIntake }
                    .addActionS(30.0) { clown.catchPixel() }
                    .addActionE(GOUPDIST + 20) { intake.status = SUpulLuiCostacu }
                    .addActionE(GOUPDIST) { clown.goUp(-1); })
            ts.at(v.cStackBackdrop[2].s(ts).so(v.dBackdropOffset * i).ce().t
                    .addActionE(30.0) { slides.setTarget(RMID_POS) })
                    .aa { clown.open() }
                    .sl(WaitPut)
        }
        ts.at(v.putPark.s(ts).t
                .addActionS(0.0) { clown.open() }
                .addActionS(50.0) { clown.goDown(); slides.setTarget(RBOT_POS) }
        )
        return ts
    }

    @JvmStatic
    fun getCycleTrajShortOld(ncycle: Int, randomCase: Int, v: ShortVals): TrajectorySequence {
        val ts = TrajectorySequence()
                .st(4)
                .aa { clown.targetPos = DiffyUpSafe }
                .at(v.aStartPreload[randomCase].t
                        .addActionT(0.1) { clown.goPreloadUp() }
                        .addActionS(60.0) { intake.status = SUpulLuiCostacu }) /// Go from start to put preload
                .aa { clown.ghearaFar?.position = ClownFDeschis }
                .aa { clown.targetPos = DiffyUpSafe }
                .sl(0.1)
        ts
                .at(v.aPreloadBackdrop.s(ts).sx(v.backdropPosX[randomCase]).t /// Set sp to last ep and set ep x
                        .addActionE(60.0) {
                            clown.targetPos = DiffyUp
                            clown.targetAngle = DiffyAUp
                            clown.curState = 0
                            clown.gelenk?.position = GelenkCenter + (if (randomCase == (if (AutoRed) 0 else 2)) -2 else 2) * GelenkDif
                        }) /// Go from put preload to backboard
                .aa { clown.open() }
                .sl(0.1)

        for (i in 0 until ncycle) {
            /// Put -> Inter1 (Diffy down) -> Inter2 -> Stack (Diffy down + gheara inchisa -> gheara deschisa + intake)
            ts.at(v.backdropStack[0].s(ts).so(v.bStackOffset * i).cb().t /// Set sp; Add offset to ep; Set "Continue begin"; Get traj
                    .addActionS(50.0) { clown.goDown(); slides.setTarget(RBOT_POS) })
                    .aa { intake.status = SIntake; }
            ts.at(v.backdropStack[1].s(ts).so(v.bStackOffset * i).cc().t) /// Set sp (with offset from last ep carried over) ; Add offset to ep; Set "Continue Continue"; Get traj
            ts.at(v.backdropStack[2].s(ts).so(v.bStackOffset * i
                    + Pose(0.0, if (i == 0) (if (AutoRed) -KMS else KMS) else 0.0, 0.0))
                    .ce().st(STACKCORRTIME1).t
                    .addActionE(80.0) { intake.status = SStack4 }
            ) /// Set sp (with offset from last ep carried over) ; Add offset to ep; Set "Continue Continue"; Get traj
                    .aa {
                        when (i) {
                            0 -> intake.status = SStack4
                            1 -> intake.status = SStack1
                            else -> intake.status = SIntake
                        }
                    }
            ts.at(v.backdropStack[3].s(ts).so(v.bStackOffset * i).ce().st(STACKCORRTIME2).t) /// Set sp (with offset from last ep carried over) ; Add offset to ep; Set "Continue Continue"; Get traj
                    .sl(WaitStack1)

            /// Stack -> Inter2 -> Inter1 -> Put (Gheara inchisa -> Diffy up + gheara deschisa)
            ts.at(v.cStackBackdrop[0].s(ts).so(v.dBackdropOffset * i).cb().t
                    .addActionT(WaitStack2) { clown.catchPixel() }
                    .addActionT(INTAKEWAIT3) { intake.status = SInvert })
                    .aa { intake.status = SIntake; clown.open() }
            ts.at(v.cStackBackdrop[1].s(ts).so(v.dBackdropOffset * i).cc().t
                    .addActionS(30.0) { clown.catchPixel() }
                    .addActionE(GOUPDIST) { clown.goUp(-1); })
            ts.at(v.cStackBackdrop[2].s(ts).so(v.dBackdropOffset * i).ce().t
                    .addActionE(20.0) { slides.setTarget(RMID_POS) })
                    .aa { clown.open() }
                    .sl(WaitPut)
        }
        ts.at(v.putPark.s(ts).t
                .addActionS(0.0) { clown.open() }
                .addActionS(50.0) { clown.goDown(); slides.setTarget(RBOT_POS) }
        )
        return ts
    }
}