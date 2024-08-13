package org.firstinspires.ftc.teamcode.auto

import com.acmerobotics.dashboard.config.Config
import org.firstinspires.ftc.teamcode.auto.AutoVars.phead
import org.firstinspires.ftc.teamcode.auto.AutoVars.pvel
import org.firstinspires.ftc.teamcode.pp.TrajCoef
import org.firstinspires.ftc.teamcode.utils.Pose
import org.firstinspires.ftc.teamcode.utils.Vec2T
import org.firstinspires.ftc.teamcode.utils.Vec2d
import org.firstinspires.ftc.teamcode.utils.Vec3
import org.firstinspires.ftc.teamcode.utils.Vec3T
import org.firstinspires.ftc.teamcode.utils.Vec4PP
import org.firstinspires.ftc.teamcode.utils.Vec4T
import kotlin.math.PI


/**
 *
 * ░░░░░██╗░█████╗░███████╗██████╗░██╗██████╗░███████╗███╗░░██╗
 * ░░░░░██║██╔══██╗██╔════╝██╔══██╗██║██╔══██╗██╔════╝████╗░██║
 * ░░░░░██║██║░░██║█████╗░░██████╦╝██║██║░░██║█████╗░░██╔██╗██║
 * ██╗░░██║██║░░██║██╔══╝░░██╔══██╗██║██║░░██║██╔══╝░░██║╚████║
 * ╚█████╔╝╚█████╔╝███████╗██████╦╝██║██████╔╝███████╗██║░╚███║
 * ░╚════╝░░╚════╝░╚══════╝╚═════╝░╚═╝╚═════╝░╚══════╝╚═╝░░╚══╝
 *
 */

class LongVals(
    @JvmField var aStartPreload: Vec3T,
    @JvmField var bPreloadStack: Vec3T,
    @JvmField var bStackBackdrop: Vec2T,
    @JvmField var cBackdropOffset: Pose,
    @JvmField var cBackdropPosX: Vec3,
    @JvmField var cBackdropStack: Vec2T,
    @JvmField var stackOffset: Pose,
    @JvmField var xBackdropStack: Vec3T,
    @JvmField var xStackBackdrop: Vec3T,
    @JvmField var zBackdropPark: TrajCoef,
    @JvmField var zzAPreload: Vec3T,
    @JvmField var zzBStack: TrajCoef,
    @JvmField var zzCStackBackdrop: Vec3T,
    @JvmField var zzDBackdropStack: Vec3T,
)

class ShortVals(
    @JvmField var aStartPreload: Vec3T,
    @JvmField var aPreloadBackdrop: TrajCoef,
    @JvmField var backdropPosX: Vec3,
    @JvmField var backdropStack: Vec4T,
    @JvmField var bStackOffset: Pose,
    @JvmField var cStackBackdrop: Vec3T,
    @JvmField var dBackdropOffset: Pose,
    @JvmField var putPark: TrajCoef,
    @JvmField var zMidStack: TrajCoef,
    @JvmField var zMidStack2: TrajCoef,
    @JvmField var zMidBackdrop: TrajCoef,
    @JvmField var zzAntiBackdropStack: Vec4T,
    @JvmField var zzAntiStackBackdrop: Vec3T,
)

@Config
object Auto {
    @JvmField
    var longBlue = LongVals(
        aStartPreload = Vec3T(
            TrajCoef(Pose(-79.0, -11.0, 1.4), phead, pvel, Vec2d(40.0, 50.0)),
            TrajCoef(Pose(-102.0, 32.0, 1.8), phead, pvel, Vec2d(40.0, 50.0)),
            TrajCoef(Pose(-110.0, 29.0, 2.7), phead, pvel, Vec2d(40.0, 50.0))),
        bPreloadStack = Vec3T(
            TrajCoef(Pose(-129.0, 44.1, 1.53), Vec2d(), Vec2d(30.0, -2.0), Vec2d(0.2, 0.3), 0.95, Vec2d(50.0, 50.0)),
            TrajCoef(Pose(-129.0, 44.1, 1.53), Vec2d(), Vec2d(30.0, -2.0), Vec2d(0.2, 0.3), .95, Vec2d(50.0, 50.0)),
            TrajCoef(Pose(-129.0, 44.1, 1.53), Vec2d(30.0, -2.0), Vec2d(30.0, -2.0), Vec2d(0.2, 0.3), .95, Vec2d(50.0, 50.0))),
        bStackBackdrop = Vec2T(
            TrajCoef(Pose(-135.0, -138.5, 1.57), Vec2d(0.7, 0.7), 0.95),
            TrajCoef(Pose(-75.0, -233.5, 1.50), Vec2d(62.0, -1.57), Vec2d(37.0, 1.57), Vec2d(0.0, 0.1), 0.9, Vec2d(50.0, 60.0))),
        cBackdropOffset = Pose(0.0, 0.3, 0.0),
        cBackdropPosX = Vec3(-59.0, -67.0, -78.7),
        cBackdropStack = Vec2T(
            TrajCoef(Pose(-132.0, -163.0, 1.63), Vec2d(40.0, 1.57), Vec2d(60.0, -1.3), 0.9),
            TrajCoef(Pose(-128.0, 45.0, 1.57), Vec2d(30.0, -1.57), Vec2d(), 0.9, Vec2d(66.0, 80.0))),
        stackOffset = Pose(1.1, 0.5, 0.0),
        xBackdropStack = Vec3T(
            TrajCoef(Pose(-126.0, -163.0, 1.57), Vec2d(30.0, 1.57), Vec2d(60.0, -1.3), 0.9),
            TrajCoef(Pose(-127.0, 7.0, 1.57), 0.9, Vec2d(0.0, 0.0)),
            TrajCoef(Pose(-93.0, 48.0, 1.57), 0.9, Vec2d(50.0, 50.0))),
        xStackBackdrop = Vec3T(
            TrajCoef(Pose(-130.0, 7.0, 1.57), 0.9),
            TrajCoef(Pose(-129.0, -133.0, 1.57), 0.9),
            TrajCoef(Pose(-72.0, -224.0, 1.57), Vec2d(40.0, -1.57), Vec2d(23.0, 1.57), 0.9, Vec2d(35.0, 60.0))),
        zBackdropPark = TrajCoef(Pose(-40.0, -233.0, 1.57), Vec2d(30.0, 1.1), Vec2d(30.0, 1.1), 0.8),
        zzAPreload = Vec3T(
            TrajCoef(Pose(-68.0, -10.0, 1.57), phead, pvel, Vec2d(40.0, 40.0)),
            TrajCoef(Pose(-73.0, 0.0, 0.8), phead, pvel, Vec2d(40.0, 40.0)),
            TrajCoef(Pose(-53.0, 10.0, -0.1), phead, pvel, Vec2d(40.0, 40.0))),
        zzBStack = TrajCoef(Pose(-70.0, 42.5, 1.57), Vec2d(0.1, 0.3), 0.9, Vec2d(40.0, 40.0)),
        zzCStackBackdrop = Vec3T(
            TrajCoef(Pose(-20.0, -48.0, 1.57), Vec2d(20.0, -1.57), Vec2d(60.0, 1.62), 0.8),
            TrajCoef(Pose(-21.0, -160.0, 1.57), Vec2d(0.5, 0.55), 0.8),
            TrajCoef(Pose(-70.0, -232.0, 1.57),  Vec2d(20.0, -1.57), Vec2d(),0.9, Vec2d(45.0, 45.0))),
        zzDBackdropStack = Vec3T(
            TrajCoef(Pose(-21.0, -110.0, 1.57), Vec2d(30.0, 1.4), Vec2d(70.0, -1.6), Vec2d(0.0, 0.0), 0.8),
            TrajCoef(Pose(-22.0, -30.0, 1.57), Vec2d(), 0.8),
            TrajCoef(Pose(-74.0, 39.0, 1.57), Vec2d(20.0, 1.6), Vec2d(), 0.9, Vec2d(65.0, 65.0)))
    )

    @JvmField
    var longRed = LongVals(
        aStartPreload = Vec3T(
            TrajCoef(Pose(-80.0, 6.0, -1.4), phead, pvel, Vec2d(34.0, 50.0)),
            TrajCoef(Pose(-105.0, -31.0, -1.8), phead, pvel, Vec2d(20.0, 80.0)),
            TrajCoef(Pose(-110.0, -34.0, -2.7), phead, pvel, Vec2d(34.0, 50.0))),
        bPreloadStack = Vec3T(
            TrajCoef(Pose(-132.0, -46.4, -1.57), Vec2d(), Vec2d(10.0, 2.0), Vec2d(0.2, 0.3), 0.95, Vec2d(40.0, 40.0)),
            TrajCoef(Pose(-132.0, -46.4, -1.57), Vec2d(), Vec2d(20.0, 2.0), Vec2d(0.2, 0.3), 0.95, Vec2d(40.0, 40.0)),
            TrajCoef(Pose(-132.0, -46.4, -1.57), Vec2d(30.0, 2.0), Vec2d(30.0, 2.0), Vec2d(0.2, 0.3), 0.95, Vec2d(40.0, 40.0))),
        bStackBackdrop = Vec2T(
            TrajCoef(Pose(-135.0, 131.0, -1.57), Vec2d(0.6, 0.6), 0.9, Vec2d(0.0, 0.1)),
            TrajCoef(Pose(-98.0, 228.0, -1.57), Vec2d(62.0, 1.57), Vec2d(40.0, -1.56), Vec2d(0.0, 0.1), 0.9, Vec2d(60.0, 60.0))),
        cBackdropOffset = Pose(0.0, 0.4, 0.0),
        cBackdropPosX = Vec3(-72.0, -81.0, -92.0),
        cBackdropStack = Vec2T(
            TrajCoef(Pose(-137.0, 186.0, -1.57), Vec2d(10.0, -1.7), Vec2d(10.0, 1.57), 0.9),
            TrajCoef(Pose(-128.0, -47.0, -1.57), Vec2d(), Vec2d(20.0, -2.7), 0.9, Vec2d(60.0, 80.0))),
        stackOffset = Pose(-1.8, -0.4, 0.0),
        xBackdropStack = Vec3T(
            TrajCoef(Pose(-130.0, 166.0, -1.57), Vec2d(30.0, -1.57), Vec2d(60.0, 1.3), 0.9),
            TrajCoef(Pose(-129.0, -38.0, -1.57), 0.9, Vec2d(0.0, 0.0)),
            TrajCoef(Pose(-97.0, -46.0, -1.57), 0.9, Vec2d(60.0, 60.0))),
        xStackBackdrop = Vec3T(
            TrajCoef(Pose(-135.0, -4.0, -1.57), 0.9),
            TrajCoef(Pose(-136.0, 136.0, -1.57), 0.9),
            TrajCoef(Pose(-74.0, 225.0, -1.57), Vec2d(40.0, 1.57), Vec2d(23.0, -1.57), 0.9, Vec2d(50.0, 60.0))),
        //zBackdropPark = TrajCoef(Pose(-135.0, 230.0, -1.57), Vec2d(30.0, -1.9), Vec2d(30.0, -1.9), 0.7),
        zBackdropPark = TrajCoef(Pose(-30.0, 227.0, -1.57),  Vec2d(20.0, -1.2), Vec2d(30.0, -1.2),0.8),
        zzAPreload = Vec3T(
            TrajCoef(Pose(-80.0, 4.0, -1.57), phead, pvel, Vec2d(40.0, 40.0)),
            TrajCoef(Pose(-75.0, -6.0, -0.8), phead, pvel, Vec2d(40.0, 40.0)),
            TrajCoef(Pose(-59.0, -16.0, 0.1), phead, pvel, Vec2d(40.0, 40.0))),
        zzBStack = TrajCoef(Pose(-72.0, -45.0, -1.57), Vec2d(0.1, 0.3), 0.9, Vec2d(40.0, 40.0)),
        zzCStackBackdrop = Vec3T(
            TrajCoef(Pose(-12.0, 20.0, -1.57), Vec2d(20.0, 1.57), Vec2d(60.0, -1.62), 0.9),
            TrajCoef(Pose(-10.0, 154.0, -1.57), Vec2d(0.5, 0.55), 0.9),
            TrajCoef(Pose(-72.0, 227.0, -1.57),  Vec2d(20.0, 1.57), Vec2d(30.0, -1.57),0.9, Vec2d(70.0, 70.0))),
        zzDBackdropStack = Vec3T(
            TrajCoef(Pose(-18.0, 104.0, -1.57), Vec2d(30.0, -1.4), Vec2d(70.0, 1.6), Vec2d(0.0, 0.0), 0.9),
            TrajCoef(Pose(-21.0, 24.0, -1.57), Vec2d(), 0.9),
            TrajCoef(Pose(-71.0, -47.0, -1.57), Vec2d(20.0, -1.6), Vec2d(), 0.9, Vec2d(70.0, 70.0)))
    )

    @JvmField
    var shortBlue = ShortVals(
        aStartPreload = Vec3T(
            TrajCoef(Pose(-78.0, 7.0, -1.58), phead, pvel, Vec2d(60.0, 60.0)),
            TrajCoef(Pose(-69.0, 3.0, -0.2), phead, pvel, Vec2d(60.0, 60.0)),
            TrajCoef(Pose(-40.0, -15.0, 0.0), phead, pvel, Vec2d(60.0, 60.0))),
        aPreloadBackdrop = TrajCoef(Pose(0.0, -90.0, 1.57), Vec2d(0.2, 0.4), 0.9, Vec2d(30.0, 60.0)),
        backdropPosX = Vec3(-78.0, -71.0, -63.0),
        backdropStack = Vec4T(
            TrajCoef(Pose(-15.0, -4.0, 1.57), Vec2d(20.0, 1.57), Vec2d(10.0, 1.64), 0.9),
            TrajCoef(Pose(-15.0, 100.0, 1.57), Vec2d(5.0, -0.7), Vec2d(0.0, 0.0), 0.9),
            TrajCoef(Pose(-78.0, 186.0, 1.59), Vec2d(40.0, 1.57), Vec2d(30.0, -1.6), 0.9, Vec2d(50.0, 50.0)),
            TrajCoef(Pose(-103.0, 187.9, 1.59), Vec2d(40.0, 1.57), Vec2d(60.0, 0.1), 0.9, Vec2d(60.0, 60.0))),
        bStackOffset = Pose(),
        cStackBackdrop = Vec3T(
            TrajCoef(Pose(-22.0, 135.0, 1.57), Vec2d(15.0, -2.2), Vec2d(30.0, 1.6), Vec2d(0.2, 0.4), 0.9),
            TrajCoef(Pose(-22.0, -10.0, 1.57), Vec2d(0.5, 0.7), 0.9),
            TrajCoef(Pose(-60.0, -88.0, 1.57), Vec2d(20.0, -1.5), Vec2d(40.0, 1.5), Vec2d(0.0, 0.1), 0.9, Vec2d(50.0, 54.0))),
        dBackdropOffset = Pose(),
        putPark = TrajCoef(Pose(-14.0, -90.0, 1.57), Vec2d(20.0, 1.5), Vec2d(20.0, 1.5), 0.8),
        zMidStack = TrajCoef(Pose(-77.0, 186.0, 1.57), 0.9, Vec2d(60.0, 60.0)),
        zMidStack2 = TrajCoef(Pose(-100.0, 188.0, 1.52), Vec2d(130.0, 1.57), Vec2d(50.0, -0.2), Vec2d(0.7, 0.8), 0.9, Vec2d(50.0, 50.0)),
        zMidBackdrop = TrajCoef(Pose(-76.0, -87.0, 1.57), 0.9, Vec2d(50.0, 50.0)),
        zzAntiBackdropStack = Vec4T(
            TrajCoef(Pose(-138.0, 0.0, 1.57), 0.9),
            TrajCoef(Pose(-139.0, 184.0, 1.57), 0.9, Vec2d(60.0, 80.0)),
            TrajCoef(Pose(-139.0, 150.0, 1.57), 0.9),
            TrajCoef(Pose(-110.0, 190.0, 1.57), 0.9, Vec2d(80.0, 80.0)),
        ),
        zzAntiStackBackdrop = Vec3T(
            TrajCoef(Pose(-142.0,  -20.0, 2.2), Vec2d(0.6, 0.7), 0.9),
            TrajCoef(Pose(-70.0, -85.0, 2.2), Vec2d(0.0, 0.01), 0.9, Vec2d(50.0, 50.0)),
            TrajCoef(Pose()),
        )
    )

    @JvmField
    var shortRed = ShortVals(
        aStartPreload = Vec3T(
            TrajCoef(Pose(-76.0, -7.0, 1.57), phead, pvel, Vec2d(30.0, 30.0)),
            TrajCoef(Pose(-66.0, 1.0, 0.2), phead, pvel, Vec2d(30.0, 30.0)),
            TrajCoef(Pose(-54.0, 14.0, 0.0), phead, pvel, Vec2d(30.0, 30.0))),
        aPreloadBackdrop = TrajCoef(Pose(0.0, 85.0, -1.57), Vec2d(0.2, 0.3), 0.9, Vec2d(30.0, 30.0)),
        backdropPosX = Vec3(-81.0, -79.0, -59.0),
        backdropStack = Vec4T(
            TrajCoef(Pose(-14.0, 0.0, -1.5), Vec2d(30.0, -1.57), Vec2d(20.0, 1.64), Vec2d(0.0, 0.2), 0.9),
            TrajCoef(Pose(-13.0, -100.0, -1.5), Vec2d(5.0, 0.7), Vec2d(0.0, 0.0), Vec2d(0.0, 0.1), 0.9),
            TrajCoef(Pose(-64.0, -190.0, -1.57), Vec2d(40.0, -1.57), Vec2d(20.0, 1.4), 0.9, Vec2d(70.0, 70.0)),
            TrajCoef(Pose(-96.0, -191.0, -1.57), Vec2d(50.0, -1.57), Vec2d(30.0, 0.1), 0.9, Vec2d(50.0, 50.0))),
        bStackOffset = Pose(),
        cStackBackdrop = Vec3T(
            TrajCoef(Pose(-8.0, -135.0, -1.57), Vec2d(15.0, 2.2), Vec2d(30.0, -1.6), Vec2d(0.2, 0.4), 0.9),
            TrajCoef(Pose(-8.0, 35.0, -1.57), Vec2d(0.7, 0.7), 0.9),
            TrajCoef(Pose(-62.0, 80.0, -1.57), Vec2d(20.0, 1.5), Vec2d(60.0, -1.58), Vec2d(0.0, 0.01), 0.9, Vec2d(50.0, 50.0))),
        dBackdropOffset = Pose(),
        putPark = TrajCoef(Pose(-17.0, 88.0, -1.57), Vec2d(50.0, -1.5), Vec2d(50.0, -1.5), 0.8, Vec2d(0.1, 0.2)),
        zMidStack = TrajCoef(Pose(-70.0, -190.0, -1.57), 0.9, Vec2d(60.0, 60.0)),
        zMidStack2 = TrajCoef(Pose(-97.0, -190.0, -1.52), Vec2d(180.0, -1.57), Vec2d(30.0, 0.0), Vec2d(0.7, 0.8), 0.9, Vec2d(60.0, 60.0)),
        zMidBackdrop = TrajCoef(Pose(-67.0, 81.0, -1.57), 0.8, Vec2d(60.0, 60.0)),
        zzAntiBackdropStack = Vec4T(
            TrajCoef(Pose(-137.0, 0.0, -1.57), Vec2d(30.0, -2.1), Vec2d(), 0.9),
            TrajCoef(Pose(-132.0, -190.0, -1.57), 0.9, Vec2d(60.0, 80.0)),
            TrajCoef(Pose(-135.0, -150.0, -1.57), 0.9),
            TrajCoef(Pose(-110.0, -193.0, -1.57), 0.9, Vec2d(80.0, 80.0)),
        ),
        zzAntiStackBackdrop = Vec3T(
            TrajCoef(Pose(-136.0,  0.0, -2.0), Vec2d(0.4, 0.5), 0.9),
            TrajCoef(Pose(-100.0, 85.0, -2.0), Vec2d(), Vec2d(30.0, -1.57), Vec2d(0.0, 0.0), 0.9, Vec2d(60.0, 80.0)),
            TrajCoef(Pose()),
        )
    )
}

@Config
object AutoVars {
    var phead = Vec2d(0.2, 0.4)
    var pvel = 0.9

    @JvmField
    var WaitPut = 0.02

    @JvmField
    var WaitStack1 = 0.2

    @JvmField
    var WaitStack2 = 0.2

    @JvmField
    var WaitStack2Min = 0.15

    @JvmField
    var WaitPreload = 0.05

    @JvmField
    var INTAKEWAIT2 = 0.15

    @JvmField
    var INTAKEWAIT3 = 0.15

    @JvmField
    var GOUPDISTRED = 60.0

    @JvmField
    var GOUPDISTBLUE = 65.0

    @JvmField
    var SLEEPY_TIME = 0.0

    @JvmField
    var failsafe1 = Pose(0.0, -20.0, 0.0)

    @JvmField
    var failsafe1s = Pose(0.0, -1.0, 0.0)

    @JvmField
    var failsafe2 = Pose(10.0, 4.0, 0.0)

    @JvmField
    var startPoses = Vec4PP(
        Pose(-110.0, 170.0, PI / 2),
        Pose(-110.0, -170.0, -PI / 2),
        Pose(50.0, 170.0, PI / 2),
        Pose(50.0, -170.0, -PI / 2),
    )

    val colours = arrayOf("#8EE3EF", "#AEF3E7", "#F6BD60", "#F7EDE2", "#37718E", "#F5CAC3", "#84A59D", "#F28482", "#19535F", "#0B7A75", "#D7C9AA", "#7B2D26", "#F0F3F5")
}
