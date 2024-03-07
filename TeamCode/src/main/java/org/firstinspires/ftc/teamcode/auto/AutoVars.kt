package org.firstinspires.ftc.teamcode.auto

import com.acmerobotics.dashboard.config.Config
import org.firstinspires.ftc.teamcode.pp.PP.MAX_FRACTION
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
                    Pose(-80.0, 2.0, 1.4),
                    Vec2d(), Vec2d(60.0, 1.5),
                    Vec2d(0.3, 0.9), 0.8,
                    Vec2d(34.0, 50.0)
            ),
            TrajCoef(
                    Pose(-98.0, 40.0, 1.8),
                    Vec2d(60.0, 1.8), Vec2d(0.0, 0.0),
                    Vec2d(0.0, 0.8), 0.8,
                    Vec2d(20.0, 80.0)
            ),
            TrajCoef(
                    Pose(-100.0, 54.0, 2.1),
                    Vec2d(40.0, 1.7), Vec2d(0.0, 0.0),
                    Vec2d(0.0, 0.6), 1.0,
                    Vec2d(34.0, 50.0)
            )
    )

    @JvmField
    var bPStack = Vec3T(
            TrajCoef(
                    Pose(-150.0, 74.0, 1.57),
                    Vec2d(70.0, -2.0), Vec2d(60.0, 2.0),
                    Vec2d(0.2, 0.7),
                    0.8
            ),

            TrajCoef(
                    Pose(-150.0, 74.0, 1.57),
                    Vec2d(60.0, 3.1), Vec2d(20.0, 2.0),
                    0.8
            ),

            TrajCoef(
                    Pose(-150.0, 74.0, 1.57),
                    Vec2d(100.0, -2.0), Vec2d(100.0, -0.8),
                    0.8, Vec2d(50.0, 120.0)
            )
    )

    @JvmField
    var bPAfterShave = TrajCoef(
            Pose(-126.0, 61.0, 1.40),
            1.5, Vec2d(999.0, 1000.0)
    )


    @JvmField
    var bPutPos = TrajCoef(
            Pose(), Pose(0.0, -226.0, 1.59),
            Vec2d(200.0, 4.3), Vec2d(80.0, 2.1),
            Vec2d(0.0, 0.1), 1.0, Vec2d(40.0, 100.0)
    )

    @JvmField
    var bPutXCase = Vec4(-46.0, -70.0, -81.0, -74.0)

    @JvmField
    var bPutYOffsetCase = Vec3(0.0, 0.0, 0.0)

    @JvmField
    var bStackOffset = Pose(0.0, 0.0, 0.0)

    @JvmField
    var bPutOffset = Pose(0.0, 0.0, 0.0)

    @JvmField
    var bStackPos2 = TrajCoef(
            Pose(), Pose(-137.0, 80.0, 1.50),
            Vec2d(80.0, -3.6), Vec2d(100.0, -1.4),
            Vec2d(), 1.0, Vec2d(40.0, 70.0)
    )

    @JvmField
    var bPAfterAfterShave = TrajCoef(
            Pose(), Pose(-110.0, 59.0, 1.40),
            1.7, Vec2d(999.0, 1000.0)
    )

    @JvmField
    var bParkPos = TrajCoef(
            Pose(-123.0, -237.0, 1.57),
            Vec2d(60.0, 1.9), Vec2d(60.0, 1.9),
            0.7
    )
}

@Config
object RedLongP {
    @JvmField
    var rPPos = Vec3T( /// Pos 0 = under thruss, 1 = mid, 2 e odar
            TrajCoef(
                    Pose(-80.0, -4.0, -1.4),
                    Vec2d(), Vec2d(60.0, -1.5),
                    Vec2d(0.3, 0.9), 0.8,
                    Vec2d(34.0, 50.0)
            ),
            TrajCoef(
                    Pose(-103.0, -40.0, -1.8),
                    Vec2d(60.0, -1.8), Vec2d(0.0, 0.0),
                    Vec2d(0.0, 0.8), 0.8,
                    Vec2d(20.0, 80.0)
            ),
            TrajCoef(
                    Pose(-100.0, -54.0, -2.1),
                    Vec2d(40.0, -1.7), Vec2d(0.0, 0.0),
                    Vec2d(0.0, 0.6), 1.0,
                    Vec2d(34.0, 50.0)
            )
    )

    @JvmField
    var rPStack = Vec3T(
            TrajCoef(
                    Pose(-150.0, -75.0, -1.57),
                    Vec2d(30.0, -1.7), Vec2d(50.0, 2.0),
                    Vec2d(0.2, 0.7),
                    0.8
            ),

            TrajCoef(
                    Pose(-150.0, -74.0, -1.57),
                    Vec2d(100.0, -3.5), Vec2d(20.0, 2.0),
                    0.8
            ),

            TrajCoef(
                    Pose(-150.0, -74.0, -1.57),
                    Vec2d(100.0, 2.0), Vec2d(20.0, 2.0),
                    0.8, Vec2d(50.0, 120.0)
            )
    )

    @JvmField
    var rPAfterShave = TrajCoef(
            Pose(-123.0, -56.0, -1.50),
            1.7, Vec2d(999.0, 1000.0)
    )


    @JvmField
    var rPutPos = TrajCoef(
            Pose(), Pose(0.0, 225.0, -1.59),
            Vec2d(200.0, -4.5), Vec2d(80.0, -2.1),
            Vec2d(0.0, 0.1), 1.0, Vec2d(40.0, 100.0)
    )

    @JvmField
    var rPutXCase = Vec4(-46.0, -66.0, -81.0, -72.0)

    @JvmField
    var rPutYOffsetCase = Vec3(0.0, 0.0, 0.0)

    @JvmField
    var rStackOffset = Pose(0.0, 0.0, 0.0)

    @JvmField
    var rPutOffset = Pose(0.0, 0.0, 0.0)

    @JvmField
    var rStackPos2 = TrajCoef(
            Pose(), Pose(-137.0, -76.0, -1.57),
            Vec2d(80.0, 3.3), Vec2d(100.0, 1.7),
            Vec2d(), 1.0, Vec2d(40.0, 70.0)
    )

    @JvmField
    var rPAfterAfterShave = TrajCoef(
            Pose(), Pose(-120.0, -57.0, -1.57),
            1.7, Vec2d(999.0, 1000.0)
    )

    @JvmField
    var rParkPos = TrajCoef(
            Pose(-123.0, 237.0, -1.57),
            Vec2d(60.0, -1.9), Vec2d(60.0, -1.9),
            0.7
    )
}

@Config
object BlueShortP {
    @JvmField
    var sbPPos = Vec3T( /// Pos 0 = under thruss, 1 = mid, 2 e odar
            TrajCoef(
                    Pose(-75.0, -10.0, -1.57),
                    Vec2d(), Vec2d(60.0, -1.5),
                    Vec2d(0.2, 0.7), 0.8
            ),
            TrajCoef(
                    Pose(-66.0, -10.0, -0.2),
                    Vec2d(), Vec2d(2.0, 0.0),
                    Vec2d(0.0, 0.8), 0.8,
                    Vec2d(34.0, 60.0)
            ),
            TrajCoef(
                    Pose(-40.0, -30.0, 0.0), 1.0
            )
    )

    @JvmField
    var sbPutFromPreloadPos = TrajCoef(
            Pose(0.0, -111.0, 1.57), Vec2d(), Vec2d(),
            Vec2d(0.2, 0.8), 0.9, Vec2d(30.0, 60.0)
    )

    @JvmField
    var sbPutXCase = Vec3(-90.0, -71.0, -59.0)

    @JvmField
    var sbPutYOffsetCase = Vec3(0.0, 0.0, 0.0)

    @JvmField
    var sbStackPPose = Vec4T(
            TrajCoef(
                    Pose(-16.0, -10.0, 1.57),
            ),
            TrajCoef(
                    Pose(-16.0, 110.0, 1.57),
                    Vec2d(5.0, 0.7), Vec2d(0.0, 0.0)
            ),
            TrajCoef(
                    Pose(-90.0, 164.0, 1.57),
                    Vec2d(60.0, 1.57), Vec2d(10.0, 4.0),
                    1.0, Vec2d(40.0, 90.0)
            ),
            TrajCoef(
                    Pose(-70.0, 160.0, 1.57),
                    1.0, Vec2d(99999.0, 100000.0)
            )
    )

    @JvmField
    var sbStackPPut = Vec3T(
            TrajCoef(
                    Pose(-20.0, 130.0, 1.57),
                    Vec2d(10.0, 4.0), Vec2d(60.0, 1.57)
            ),
            TrajCoef(
                    Pose(-22.0, -10.0, 1.57),
                    Vec2d(5.0, -0.7), Vec2d(0.0, 0.0)
            ),
            TrajCoef(
                    Pose(-56.0, -100.0, 1.57),
                    Vec2d(10.0, -1.57), Vec2d(10.0, -4.0),
                    1.0, Vec2d(30.0, 60.0)
            )
    )

    @JvmField
    var sbStackOffset = Pose(0.0, 0.0, 0.0)

    @JvmField
    var sbPutOffset = Pose(0.0, 0.0, 0.0)

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
            Pose(0.0, 99.0, -1.57), Vec2d(), Vec2d(),
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
                    Pose(-20.0, -171.0, -1.57),
                    Vec2d(60.0, -1.57), Vec2d(10.0, -4.0),
                    Vec2d(0.4, 0.9), 1.0, Vec2d(40.0, 90.0)
            ),
            TrajCoef(
                    Pose(-70.0, -174.0, -1.57),
            )
    )

    @JvmField
    var srStackPPut = Vec3T(
            TrajCoef(
                    Pose(-5.0, -150.0, -1.77),
                    Vec2d(60.0, -2.2), Vec2d(00.0, -1.57),
                    Vec2d(0.2, 0.4), 1.0
            ),
            TrajCoef(
                    Pose(-10.0, 50.0, -1.57),
                    Vec2d(0.0, 0.7), Vec2d(0.0, 0.0), 1.0
            ),
            TrajCoef(
                    Pose(-61.0, 100.0, -1.57),
                    Vec2d(0.0, 1.57), Vec2d(40.0, -2.0),
                    1.0, Vec2d(30.0, 60.0)
            )
    )

    @JvmField
    var srStackOffset = Pose(-2.0, 1.0, 0.0)

    @JvmField
    var srPutOffset = Pose(-3.0, 1.0, 0.0)

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
    var Wait2 = 0.4

    @JvmField
    var WaitIntake = 0.5

    @JvmField
    var WaitPut = 0.25

    @JvmField
    var WaitStack1 = 0.1
    @JvmField
    var WaitStack15 = 0.3
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
