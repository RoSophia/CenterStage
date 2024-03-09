package org.firstinspires.ftc.teamcode.auto

import com.acmerobotics.dashboard.config.Config
import org.firstinspires.ftc.teamcode.pp.TrajCoef
import org.firstinspires.ftc.teamcode.utils.Pose
import org.firstinspires.ftc.teamcode.utils.Vec2d
import org.firstinspires.ftc.teamcode.utils.Vec3
import org.firstinspires.ftc.teamcode.utils.Vec3T
import org.firstinspires.ftc.teamcode.utils.Vec4
import org.firstinspires.ftc.teamcode.utils.Vec4T

@Config
object BlueLongP {
    @JvmField
    var bPPos = Vec3T( /// Pos 0 = under thruss, 1 = mid, 2 e odar
            TrajCoef(
                    Pose(-80.0, 12.0, 1.4),
                    Vec2d(), Vec2d(60.0, 1.5),
                    Vec2d(0.3, 0.9), 0.8,
                    Vec2d(34.0, 50.0)
            ),
            TrajCoef(
                    Pose(-101.0, 40.0, 1.8),
                    Vec2d(60.0, 1.8), Vec2d(0.0, 0.0),
                    Vec2d(0.0, 0.8), 0.8,
                    Vec2d(20.0, 80.0)
            ),
            TrajCoef(
                    Pose(-100.0, 64.0, 2.1),
                    Vec2d(40.0, 1.7), Vec2d(0.0, 0.0),
                    Vec2d(0.0, 0.6), 1.0,
                    Vec2d(34.0, 50.0)
            )
    )

    @JvmField
    var bPStack = Vec3T(
            TrajCoef(
                    Pose(-136.0, 90.0, 1.57),
                    Vec2d(0.0, -2.0), Vec2d(30.0, -2.0),
                    Vec2d(0.2, 0.7),
                    0.8
            ),

            TrajCoef(
                    Pose(-136.0, 91.0, 1.57),
                    Vec2d(0.0, 3.1), Vec2d(30.0, -2.0),
                    0.8
            ),

            TrajCoef(
                    Pose(-136.0, 90.0, 1.57),
                    Vec2d(0.0, -2.0), Vec2d(30.0, -2.0),
                    0.8, Vec2d(50.0, 120.0)
            )
    )

    @JvmField
    var bPAfterShave = TrajCoef(
            Pose(-132.0, 92.0, 1.57),
            0.5, Vec2d(999.0, 1000.0)
    )


    @JvmField
    var bPut1Pos = TrajCoef(
            Pose(-136.0, -130.0, 1.57),
            1.0, Vec2d(0.0, 1.0)
    )

    @JvmField
    var bPutPos = TrajCoef(
            Pose(0.0, -218.0, 1.4),
            Vec2d(40.0, -1.57), Vec2d(40.0, 1.57),
            1.0, Vec2d(40.0, 100.0)
    )

    @JvmField
    var bPutXCase = Vec4(-50.0, -60.0, -70.0, -70.0)

    @JvmField
    var bPutY = -211.0

    @JvmField
    var bStackPos12 = TrajCoef(
            Pose(), Pose(-129.0, -180.0, 1.57),
            Vec2d(30.0, 1.57), Vec2d(),
            1.0
    )

    @JvmField
    var bStackPos2 = TrajCoef(
            Pose(), Pose(-145.0, 90.0, 1.3),
            1.0, Vec2d(50.0, 100.0)
    )

    @JvmField
    var bPAfterAfterShave = TrajCoef(
            Pose(), Pose(-116.0, 71.0, 1.3),
            0.8, Vec2d(999.0, 1000.0)
    )

    @JvmField
    var bParkPos = TrajCoef(
            Pose(-123.0, -237.0, 1.57),
            Vec2d(60.0, 1.9), Vec2d(60.0, 1.9),
            0.7
    )

    @JvmField
    var bPutYOffsetCase = Vec3(0.0, 0.0, 0.0)

    @JvmField
    var bStackOffset = Pose(0.0, 0.0, 0.0)

    @JvmField
    var bPutOffset = Pose(0.0, -1.0, 0.0)

}

@Config
object RedLongP {
    @JvmField
    var rPPos = Vec3T(
            /// Pos 0 = under thruss, 1 = mid, 2 e odar
            TrajCoef(
                    Pose(-80.0, -12.0, -1.4),
                    Vec2d(), Vec2d(60.0, -1.5),
                    Vec2d(0.3, 0.9), 0.8,
                    Vec2d(34.0, 50.0)
            ),
            TrajCoef(
                    Pose(-101.0, -40.0, -1.8),
                    Vec2d(60.0, -1.8), Vec2d(0.0, 0.0),
                    Vec2d(0.0, 0.8), 0.8,
                    Vec2d(20.0, 80.0)
            ),
            TrajCoef(
                    Pose(-100.0, -64.0, -2.1),
                    Vec2d(40.0, -1.7), Vec2d(0.0, 0.0),
                    Vec2d(0.0, 0.6), 1.0,
                    Vec2d(34.0, 50.0)
            ),
    )

    @JvmField
    var rPStack = Vec3T(
            TrajCoef(
                    Pose(-132.0, -62.0, -1.57),
                    Vec2d(0.0, 2.0), Vec2d(30.0, 2.0),
                    0.8, Vec2d(50.0, 120.0)
            ),

            TrajCoef(
                    Pose(-132.0, -62.0, -1.57),
                    Vec2d(0.0, 2.0), Vec2d(30.0, 2.0),
                    Vec2d(0.2, 0.7),
                    0.8
            ),
            TrajCoef(
                    Pose(-132.0, -63.0, -1.57),
                    Vec2d(30.0, 2.0), Vec2d(30.0, 2.0),
                    Vec2d(0.1, 0.4), 0.8
            )

    )

    @JvmField
    var rPAfterShave = TrajCoef(
            Pose(-132.0, -92.0, -1.57),
            0.5, Vec2d(999.0, 1000.0)
    )

    @JvmField
    var rPut1Pos = TrajCoef(
            Pose(-136.0, 130.0, -1.57),
            1.0, Vec2d(0.0, 1.0)
    )

    @JvmField
    var rPutPos = TrajCoef(
            Pose(0.0, 210.0, -1.4),
            Vec2d(40.0, 1.57), Vec2d(40.0, -1.56),
            1.0, Vec2d(40.0, 100.0)
    )

    @JvmField
    var rPutXCase = Vec4(-58.0, -67.0, -75.0, -70.0)

    @JvmField
    var rPutY = 213.0

    @JvmField
    var rStackPos12 = TrajCoef(
            Pose(), Pose(-129.0, 180.0, -1.57),
            Vec2d(30.0, -1.57), Vec2d(),
            1.0
    )

    @JvmField
    var rStackPos2 = TrajCoef(
            Pose(), Pose(-162.0, -99.0, -1.2),
            1.0, Vec2d(80.0, 100.0)
    )

    @JvmField
    var rPAfterAfterShave = TrajCoef(
            Pose(), Pose(-136.0, -70.0, -1.2),
            1.1, Vec2d(999.0, 1000.0)
    )

    @JvmField
    var rParkPos = TrajCoef(
            Pose(-134.0, 237.0, -1.57),
            Vec2d(60.0, -1.9), Vec2d(60.0, -1.9),
            0.7
    )

    @JvmField
    var rPutYOffsetCase = Vec3(0.0, 0.0, 0.0)

    @JvmField
    var rStackOffset = Pose(0.0, 0.0, 0.0)

    @JvmField
    var rPutOffset = Pose(0.0, 1.0, 0.0)

}

@Config
object BlueShortP {
    @JvmField
    var sbTrajPreload2Mid = Vec3T( /// Pos 0 = under thruss, 1 = mid, 2 e odar
            TrajCoef(
                    Pose(-75.0, -14.0, -1.57),
                    Vec2d(), Vec2d(60.0, -1.5),
                    Vec2d(0.2, 0.7), 0.8
            ),
            TrajCoef(
                    Pose(-66.0, -10.0, -0.2),
                    Vec2d(), Vec2d(2.0, -0.0),
                    Vec2d(0.0, 0.8), 0.8,
                    Vec2d(34.0, 60.0)
            ),
            TrajCoef(
                    Pose(-40.0, -30.0, 0.0), 1.0
            )
    )

    @JvmField
    var sbTrajMid2BackBoard = TrajCoef(
            Pose(0.0, -105.0, 1.57), Vec2d(), Vec2d(),
            Vec2d(0.2, 0.8), 0.9, Vec2d(30.0, 60.0)
    )

    @JvmField
    var sbPutXCase = Vec3(-86.0, -68.0, -55.0)

    @JvmField
    var sbPutYOffsetCase = Vec3(0.0, 0.0, 0.0)

    @JvmField
    var sbTrajBack2Sstack = Vec4T(
            TrajCoef(
                    Pose(-10.0, -10.0, 1.57),
            ),
            TrajCoef(
                    Pose(-10.0, 110.0, 1.57),
                    Vec2d(5.0, -0.7), Vec2d(0.0, 0.0)
            ),
            TrajCoef(
                    //fromtrusstomid211
                    Pose(-53.0, 135.0, 1.57),
                    Vec2d(60.0, 1.57), Vec2d(10.0, 4.0),
                    Vec2d(0.4, 0.9), 1.0, Vec2d(40.0, 90.0)
            ),
            TrajCoef(
                    //frommid2tostack
                    Pose(-67.0, 172.0, 1.57), 0.8
            )
    )

    @JvmField
    var sbTrajStack2Back = Vec3T(
            TrajCoef(
                    Pose(-10.0, 150.0, 1.9),
                    Vec2d(70.0, 2.2), Vec2d(00.0, 1.57),
                    Vec2d(0.2, 0.4), 1.0
            ),
            TrajCoef(
                    Pose(-14.0, -45.0, 1.57),
                    Vec2d(0.0, -0.7), Vec2d(0.0, 0.0), 1.0
            ),
            TrajCoef(
                    Pose(-56.0, -104.0, 1.57),
                    Vec2d(30.0, -1.57), Vec2d(30.0, 1.7),
                    1.0, Vec2d(30.0, 60.0)
            )
    )

    @JvmField
    var sbStackOffset = Pose(-0.3, -1.0, 0.0)

    @JvmField
    var sbPutOffset = Pose(-0.3, -1.0, 0.0)

    @JvmField
    var sbParkPos = TrajCoef(
            Pose(-16.0, -100.0, 1.57),
            Vec2d(60.0, 0.5), Vec2d(60.0, 2.7)
    )
}

@Config
object RedShortP {
    @JvmField
    var srPPos = Vec3T( /// Pos 0 = under thruss, 1 = mid, 2 e odar
            TrajCoef(
                    Pose(-75.0, 10.0, 1.57),
                    Vec2d(), Vec2d(60.0, 1.5),
                    Vec2d(0.2, 0.7), 0.8
            ),
            TrajCoef(
                    Pose(-66.0, 10.0, 0.2),
                    Vec2d(), Vec2d(2.0, 0.0),
                    Vec2d(0.0, 0.8), 0.8,
                    Vec2d(34.0, 60.0)
            ),
            TrajCoef(
                    Pose(-40.0, 30.0, 0.0), 1.0
            )
    )

    @JvmField
    var srPutFromPreloadPos = TrajCoef(
            Pose(0.0, 101.0, -1.57), Vec2d(), Vec2d(),
            Vec2d(0.2, 0.8), 0.9, Vec2d(30.0, 60.0)
    )

    @JvmField
    var srPutXCase = Vec3(-90.0, -71.0, -59.0)

    @JvmField
    var srPutYOffsetCase = Vec3(0.0, 0.0, 0.0)

    @JvmField
    var srStackPPose = Vec4T(
            TrajCoef(
                    Pose(-16.0, 10.0, -1.57),
            ),
            TrajCoef(
                    Pose(-16.0, -110.0, -1.57),
                    Vec2d(5.0, 0.7), Vec2d(0.0, 0.0)
            ),
            TrajCoef(
                    Pose(-40.0, -140.0, -1.57),
                    Vec2d(60.0, -1.57), Vec2d(10.0, -4.0),
                    Vec2d(0.4, 0.9), 1.0, Vec2d(40.0, 90.0)
            ),
            TrajCoef(
                    Pose(-60.0, -174.0, -1.57), 0.8
            )
    )

    @JvmField
    var srStackPPut = Vec3T(
            TrajCoef(
                    Pose(-5.0, -150.0, -1.77),
                    Vec2d(70.0, -2.2), Vec2d(00.0, -1.57),
                    Vec2d(0.2, 0.4), 1.0
            ),
            TrajCoef(
                    Pose(-6.0, 45.0, -1.57),
                    Vec2d(0.0, 0.7), Vec2d(0.0, 0.0), 1.0
            ),
            TrajCoef(
                    Pose(-61.0, 100.0, -1.57),
                    Vec2d(30.0, 1.57), Vec2d(30.0, -1.7),
                    1.0, Vec2d(30.0, 60.0)
            )
    )

    @JvmField
    var srStackOffset = Pose(-0.3, 1.0, 0.0)

    @JvmField
    var srPutOffset = Pose(-0.3, 1.0, 0.0)

    @JvmField
    var srParkPos = TrajCoef(
            Pose(-16.0, 100.0, -1.57),
            Vec2d(60.0, -0.5), Vec2d(60.0, -2.7)
    )
}

@Config
object AutoVars {
    @JvmField
    var NumCycles = 3

    @JvmField
    var TimeoutWaitFirstStack = 3.5

    @JvmField
    var Min3Pixel = 0.1

    @JvmField
    var Min3PixelSpeed = 0.1

    @JvmField
    var Wait2 = 0.4

    @JvmField
    var WaitIntake = 0.5

    @JvmField
    var WaitPut = 0.25

    @JvmField
    var WaitStack1 = 0.1

    @JvmField
    var WaitStack15 = 0.5

    @JvmField
    var WaitStack2 = 0.2

    @JvmField
    var WaitStack3 = 0.2

    @JvmField
    var WaitAfterShave = 0.5

    @JvmField
    var WaitPreload = 0.4

    val colours = arrayOf("#8EE3EF", "#AEF3E7", "#F6BD60", "#F7EDE2", "#37718E", "#F5CAC3", "#84A59D", "#F28482", "#19535F", "#0B7A75", "#D7C9AA", "#7B2D26", "#F0F3F5")
}
