package org.firstinspires.ftc.teamcode.auto

import com.acmerobotics.dashboard.config.Config
import org.firstinspires.ftc.teamcode.pp.TrajCoef
import org.firstinspires.ftc.teamcode.utils.Pose
import org.firstinspires.ftc.teamcode.utils.Vec2T
import org.firstinspires.ftc.teamcode.utils.Vec2d
import org.firstinspires.ftc.teamcode.utils.Vec3
import org.firstinspires.ftc.teamcode.utils.Vec3T
import org.firstinspires.ftc.teamcode.utils.Vec4T

class LongVals(
        @JvmField var aStartPreload: Vec3T,
        @JvmField var bPreloadStack: Vec3T,
        @JvmField var bStackBackdrop: Vec2T,
        @JvmField var cBackdropOffset: Pose,
        @JvmField var cBackdropPosX: Vec3,
        @JvmField var cBackdropStack: Vec2T,
        @JvmField var stackOffset: Pose,
        @JvmField var zBackdropPark: TrajCoef)

@Config
object Longies {
    @JvmField
    var blue = LongVals(
            aStartPreload = Vec3T( /// Pos 0 = under thruss, 1 = mid, 2 e odar
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
            ),

            bPreloadStack = Vec3T(
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
            ),

            bStackBackdrop = Vec2T(
                    TrajCoef(
                            Pose(-136.0, -130.0, 1.57),
                            1.0, Vec2d(0.0, 1.0)
                    ),

                    TrajCoef(
                            Pose(-70.0, -218.0, 1.4),
                            Vec2d(40.0, -1.57), Vec2d(40.0, 1.57),
                            1.0, Vec2d(40.0, 100.0)
                    )),

            cBackdropOffset = Pose(0.0, 0.0, 0.0),

            cBackdropPosX = Vec3(-50.0, -60.0, -70.0),

            cBackdropStack = Vec2T(
                    TrajCoef(
                            Pose(), Pose(-129.0, -180.0, 1.57),
                            Vec2d(30.0, 1.57), Vec2d(),
                            1.0
                    ),

                    TrajCoef(
                            Pose(), Pose(-145.0, 90.0, 1.3),
                            1.0, Vec2d(50.0, 100.0)
                    )),

            stackOffset = Pose(0.0, 0.0, 0.0),

            zBackdropPark = TrajCoef(
                    Pose(-123.0, -237.0, 1.57),
                    Vec2d(60.0, 1.9), Vec2d(60.0, 1.9),
                    0.7
            ),
    )

    @JvmField
    var red = LongVals(
            aStartPreload = Vec3T(
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
            ),

            bPreloadStack = Vec3T(
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

            ),

            bStackBackdrop = Vec2T(
                    TrajCoef(
                            Pose(-136.0, 130.0, -1.57),
                            1.0, Vec2d(0.0, 1.0)
                    ),

                    TrajCoef(
                            Pose(-70.0, 210.0, -1.4),
                            Vec2d(40.0, 1.57), Vec2d(40.0, -1.56),
                            1.0, Vec2d(40.0, 100.0)
                    )),

            cBackdropOffset = Pose(0.0, 1.0, 0.0),

            cBackdropPosX = Vec3(-58.0, -67.0, -75.0),

            cBackdropStack = Vec2T(
                    TrajCoef(
                            Pose(), Pose(-129.0, 180.0, -1.57),
                            Vec2d(30.0, -1.57), Vec2d(),
                            1.0
                    ),

                    TrajCoef(
                            Pose(), Pose(-162.0, -99.0, -1.2),
                            1.0, Vec2d(80.0, 100.0)
                    )),

            stackOffset = Pose(0.0, 0.0, 0.0),

            zBackdropPark = TrajCoef(
                    Pose(-134.0, 237.0, -1.57),
                    Vec2d(60.0, -1.9), Vec2d(60.0, -1.9),
                    0.7
            ),
    )
}

class ShortVals(
        @JvmField var aStartPreload: Vec3T, // Are a in fata sa se alinieze frumos alfabetic
        @JvmField var aPreloadBackdrop: TrajCoef,
        @JvmField var backdropPosX: Vec3,
        @JvmField var backdropStack: Vec4T,
        @JvmField var bStackOffset: Pose,
        @JvmField var cStackBackdrop: Vec3T, // Are c in fata sa se alinieze frumos alfabetic
        @JvmField var dBackdropOffset: Pose,
        @JvmField var putPark: TrajCoef,
)

@Config
object Shorties {
    @JvmField
    var blue = ShortVals(
            /// Preload Pos
            aStartPreload = Vec3T( /// Pos 0 = under thruss, 1 = mid, 2 e odar
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
            ),

            /// Preload Backdrop
            aPreloadBackdrop = TrajCoef(
                    Pose(0.0, -105.0, 1.57), Vec2d(), Vec2d(),
                    Vec2d(0.2, 0.8), 0.9, Vec2d(30.0, 60.0)
            ),

            /// Backdrop X Pos by case
            backdropPosX = Vec3(-86.0, -68.0, -55.0),

            /// Backdrop stack
            backdropStack = Vec4T(
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
            ),

            /// Stack Offset
            bStackOffset = Pose(-0.3, -1.0, 0.0),

            /// Stack Backdrop
            cStackBackdrop = Vec3T(
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
            ),

            /// Backdrop offset
            dBackdropOffset = Pose(-0.3, -1.0, 0.0),

            /// Park pos
            putPark = TrajCoef(
                    Pose(-16.0, -100.0, 1.57),
                    Vec2d(60.0, 0.5), Vec2d(60.0, 2.7)
            )
    )

    var red = ShortVals(
            /// Preload Pos
            aStartPreload = Vec3T( /// Pos 0 = under thruss, 1 = mid, 2 e odar
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
            ),

            aPreloadBackdrop = TrajCoef(
                    Pose(0.0, 101.0, -1.57), Vec2d(), Vec2d(),
                    Vec2d(0.2, 0.8), 0.9, Vec2d(30.0, 60.0)
            ),

            backdropPosX = Vec3(-90.0, -71.0, -59.0),

            backdropStack = Vec4T(
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
            ),

            bStackOffset = Pose(0.0, 0.0, 0.0),

            cStackBackdrop = Vec3T(
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
            ),

            dBackdropOffset = Pose(0.0, 0.0, 0.0),

            putPark = TrajCoef(
                    Pose(-16.0, 100.0, -1.57),
                    Vec2d(60.0, -0.5), Vec2d(60.0, -2.7)
            )
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

    @JvmField
    var INTAKEWAIT1 = 0.2

    @JvmField
    var INTAKEWAIT2 = 0.2

    @JvmField
    var INTAKEWAIT3 = 0.2

    @JvmField
    var INTAKEWAIT4 = 0.2

    @JvmField
    var INTAKEWAIT5 = 0.2

    @JvmField
    var TimeoutTakeAftl = 0.2

    val colours = arrayOf("#8EE3EF", "#AEF3E7", "#F6BD60", "#F7EDE2", "#37718E", "#F5CAC3", "#84A59D", "#F28482", "#19535F", "#0B7A75", "#D7C9AA", "#7B2D26", "#F0F3F5")
}
