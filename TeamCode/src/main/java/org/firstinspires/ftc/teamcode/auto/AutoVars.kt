package org.firstinspires.ftc.teamcode.auto

import com.acmerobotics.dashboard.config.Config
import org.firstinspires.ftc.teamcode.pp.TrajCoef
import org.firstinspires.ftc.teamcode.pp.Trajectory
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
        @JvmField var xBackdropStack: Vec3T,
        @JvmField var xStackBackdrop: Vec3T,
        @JvmField var zBackdropPark: TrajCoef)

class ShortVals(
        @JvmField var aStartPreload: Vec3T, // Are a in fata sa se alinieze frumos alfabetic
        @JvmField var aPreloadBackdrop: TrajCoef,
        @JvmField var backdropPosX: Vec3,
        @JvmField var backdropStack: Vec4T,
        @JvmField var bStackOffset: Pose,
        @JvmField var cStackBackdrop: Vec3T, // Are c in fata sa se alinieze frumos alfabetic
        @JvmField var dBackdropOffset: Pose,
        @JvmField var xStack: TrajCoef,
        @JvmField var putPark: TrajCoef,
)

@Config
object Auto {
    @JvmField
    var longBlue = LongVals(
            //// LONG BLUE
            aStartPreload = Vec3T( /// Pos 0 = under thruss, 1 = mid, 2 e odar
                    TrajCoef(
                            Pose(-80.0, 5.0, 1.4),
                            Vec2d(), Vec2d(60.0, 1.5),
                            Vec2d(0.3, 0.8), 0.8,
                            Vec2d(34.0, 50.0)
                    ),
                    TrajCoef(
                            Pose(-104.0, 40.0, 1.8),
                            Vec2d(60.0, 1.8), Vec2d(0.0, 0.0),
                            Vec2d(0.3, 0.8), 0.8,
                            Vec2d(34.0, 50.0)
                    ),
                    TrajCoef(
                            Pose(-100.0, 54.0, 2.1),
                            Vec2d(40.0, 1.7), Vec2d(0.0, 0.0),
                            Vec2d(0.3, 0.8), 0.8,
                            Vec2d(34.0, 50.0)
                    )
            ),

            //// LONG BLUE
            bPreloadStack = Vec3T(
                    TrajCoef(
                            Pose(-129.0, 56.8, 1.59),
                            Vec2d(0.0, -2.0), Vec2d(30.0, -2.0),
                            Vec2d(0.2, 0.7),
                            0.8
                    ),

                    TrajCoef(
                            Pose(-129.0, 57.5, 1.59),
                            Vec2d(0.0, 3.1), Vec2d(30.0, -2.0),
                            Vec2d(0.2, 0.7),
                            0.8
                    ),

                    TrajCoef(
                            Pose(-130.0, 56.8, 1.59),
                            Vec2d(40.0, -1.6), Vec2d(30.0, -2.0),
                            Vec2d(0.2, 0.7),
                            0.8, Vec2d(10.0, 20.0)
                    )
            ),

            //// LONG BLUE
            bStackBackdrop = Vec2T(
                    TrajCoef(
                            Pose(-133.3, -130.0, 1.57), 0.8,
                            Vec2d(0.1, 0.4)
                    ),

                    TrajCoef(
                            Pose(-71.0, -222.5, 1.63),
                            Vec2d(40.0, -1.57), Vec2d(23.0, 1.57),
                            0.8, Vec2d(35.0, 60.0)
                    )),

            //// LONG BLUE
            cBackdropOffset = Pose(2.0, 0.0, 0.0),

            //// LONG BLUE
            cBackdropPosX = Vec3(-57.0, -61.0, -75.0),

            //// LONG BLUE
            cBackdropStack = Vec2T(
                    TrajCoef(
                            Pose(), Pose(-126.0, -160.0, 1.57),
                            Vec2d(40.0, 1.57), Vec2d(60.0, -1.3),
                            0.8
                    ),

                    TrajCoef(
                            Pose(), Pose(-125.0, 55.8, 1.57),
                            0.8, Vec2d(40.0, 80.0)
                    )),

            //// LONG BLUE
            stackOffset = Pose(2.0, 2.0, 0.0),

            //// LONG BLUE
            xBackdropStack = Vec3T(
                    TrajCoef(
                            Pose(), Pose(-126.0, -160.0, 1.57),
                            Vec2d(30.0, 1.57), Vec2d(60.0, -1.3),
                            1.3
                    ),

                    TrajCoef(
                            Pose(), Pose(-127.0, 10.0, 1.57),
                            1.3, Vec2d(0.0, 0.0)
                    ),

                    TrajCoef(
                            Pose(), Pose(-89.0, 51.0, 1.57),
                            1.3, Vec2d(30.0, 50.0)
                    ),
            ),

            //// LONG BLUE
            xStackBackdrop = Vec3T(
                    TrajCoef(
                            Pose(-130.0, 10.0, 1.57), 1.3
                    ),

                    TrajCoef(
                            Pose(-129.0, -130.0, 1.57), 1.3
                    ),

                    TrajCoef(
                            Pose(-72.0, -221.0, 1.57),
                            Vec2d(40.0, -1.57), Vec2d(23.0, 1.57),
                            1.3, Vec2d(35.0, 60.0)
                    ),
            ),

            //// LONG BLUE
            zBackdropPark = TrajCoef(
                    Pose(), Pose(-118.0, -234.0, 1.57),
                    Vec2d(60.0, 1.9), Vec2d(60.0, 1.9),
                    0.9
            ),
    )

    @JvmField
    var longRed = LongVals(
            //// LONG RED
            aStartPreload = Vec3T(
                    /// Pos 0 = under thruss, 1 = mid, 2 e odar
                    TrajCoef(
                            Pose(-80.0, -7.0, -1.4),
                            Vec2d(), Vec2d(60.0, -1.5),
                            Vec2d(0.3, 0.9), 0.8,
                            Vec2d(34.0, 50.0)
                    ),
                    TrajCoef(
                            Pose(-104.0, -40.0, -1.8),
                            Vec2d(60.0, -1.8), Vec2d(0.0, 0.0),
                            Vec2d(0.3, 0.8), 0.8,
                            Vec2d(20.0, 80.0)
                    ),
                    TrajCoef(
                            Pose(-100.0, -62.0, -2.1),
                            Vec2d(30.0, -1.7), Vec2d(0.0, 0.0),
                            Vec2d(0.3, 0.6), 0.8,
                            Vec2d(34.0, 50.0)
                    ),
            ),

            //// LONG RED
            bPreloadStack = Vec3T(
                    TrajCoef(
                            Pose(-132.0, -57.0, -1.57),
                            Vec2d(0.0, 2.0), Vec2d(20.0, 2.0),
                            0.8
                    ),

                    TrajCoef(
                            Pose(-132.0, -57.0, -1.57),
                            Vec2d(), Vec2d(20.0, 2.0),
                            Vec2d(0.2, 0.7),
                            0.8
                    ),
                    TrajCoef(
                            Pose(-132.0, -57.0, -1.57),
                            Vec2d(0.0, 1.6), Vec2d(30.0, 2.0),
                            Vec2d(0.2, 0.7), 0.76
                    )
            ),

            //// LONG RED
            bStackBackdrop = Vec2T(
                    TrajCoef(
                            Pose(-123.0, 130.0, -1.57),
                            0.8, Vec2d(0.0, 0.1)
                    ),

                    TrajCoef(
                            Pose(-86.0, 219.0, -1.57),
                            Vec2d(60.0, 1.57), Vec2d(40.0, -1.56),
                            0.8, Vec2d(40.0, 60.0)
                    )),

            //// LONG RED
            cBackdropOffset = Pose(0.0, 0.0, 0.0),

            //// LONG RED
            cBackdropPosX = Vec3(-61.0, -74.0, -86.0),

            //// LONG RED
            cBackdropStack = Vec2T(
                    TrajCoef(
                            Pose(), Pose(-134.0, 180.0, -1.57),
                            Vec2d(30.0, -1.57), Vec2d(),
                            0.8
                    ),

                    TrajCoef(
                            Pose(), Pose(-132.0, -56.5, -1.57),
                            0.8, Vec2d(40.0, 80.0)
                    )),

            //// LONG RED
            stackOffset = Pose(0.0, 0.0, 0.0),

            //// LONG RED
            xBackdropStack = Vec3T(
                    TrajCoef(
                            Pose(), Pose(-126.0, 160.0, -1.57),
                            Vec2d(30.0, 1.57), Vec2d(60.0, -1.3),
                            1.0
                    ),

                    TrajCoef(
                            Pose(), Pose(-127.0, -10.0, -1.57),
                            1.0, Vec2d(0.0, 0.0)
                    ),

                    TrajCoef(
                            Pose(), Pose(-80.0, -52.7, -1.57),
                            1.0, Vec2d(40.0, 80.0)
                    ),
            ),

            //// LONG RED
            xStackBackdrop = Vec3T(
                    TrajCoef(
                            Pose(-135.0, -10.0, -1.57), 1.0
                    ),

                    TrajCoef(
                            Pose(-136.0, 130.0, -1.57), 1.0
                    ),

                    TrajCoef(
                            Pose(-74.0, 220.0, -1.57),
                            Vec2d(40.0, -1.57), Vec2d(23.0, 1.57),
                            1.0, Vec2d(35.0, 60.0)
                    ),
            ),

            //// LONG RED
            zBackdropPark = TrajCoef(
                    Pose(-138.0, 237.0, -1.57),
                    Vec2d(60.0, -1.9), Vec2d(60.0, -1.9),
                    0.7
            ),
    )

    @JvmField
    var shortBlue = ShortVals(
            //// SHORT BLUE
            aStartPreload = Vec3T( /// Pos 0 = under thruss, 1 = mid, 2 e odar
                    TrajCoef(
                            Pose(-75.0, -12.0, -1.57),
                            Vec2d(), Vec2d(60.0, -1.5),
                            Vec2d(0.2, 0.8), 0.8
                    ),
                    TrajCoef(
                            Pose(-66.0, -10.0, -0.2),
                            Vec2d(), Vec2d(2.0, -0.0),
                            Vec2d(0.1, 0.5), 0.8,
                            Vec2d(34.0, 60.0)
                    ),
                    TrajCoef(
                            Pose(-40.0, -26.0, 0.0),
                            Vec2d(0.2, 0.6), 1.0
                    )
            ),

            //// SHORT BLUE
            aPreloadBackdrop = TrajCoef(
                    Pose(0.0, -108.0, 1.57), Vec2d(), Vec2d(),
                    Vec2d(0.2, 0.8), 0.9, Vec2d(30.0, 60.0)
            ),

            //// SHORT BLUE
            backdropPosX = Vec3(-92.0, -79.0, -58.0),

            //// SHORT BLUE
            backdropStack = Vec4T(
                    TrajCoef(
                            Pose(-16.0, -14.0, 1.5),
                            Vec2d(30.0, 1.57), Vec2d(10.0, 1.64),
                            Vec2d(0.0, 0.2), 0.8
                    ),
                    TrajCoef(
                            Pose(-13.5, 90.0, 1.57),
                            Vec2d(5.0, -0.7), Vec2d(0.0, 0.0),
                            Vec2d(0.0, 0.1), 0.8
                    ),
                    TrajCoef(
                            Pose(-72.0, 168.0, 1.5),
                            Vec2d(40.0, 1.57), Vec2d(30.0, -1.6), 0.8,
                            Vec2d(30.0, 60.0)
                    ),
                    TrajCoef(),
            ),

            //// SHORT BLUE
            bStackOffset = Pose(1.4, 1.0, 0.0),

            //// SHORT BLUE
            cStackBackdrop = Vec3T(
                    TrajCoef(
                            Pose(-17.0, 120.0, 1.57),
                            Vec2d(15.0, -2.2), Vec2d(30.0, 1.6),
                            Vec2d(0.2, 0.4), 0.8
                    ),
                    TrajCoef(
                            Pose(-17.0, -50.0, 1.57),
                            Vec2d(0.0, -0.7), Vec2d(0.0, 0.0),
                            Vec2d(0.0, 0.1), 0.8
                    ),
                    TrajCoef(
                            Pose(-63.0, -106.0, 1.57),
                            Vec2d(20.0, -1.5), Vec2d(30.0, 1.5),
                            1.0, Vec2d(40.0, 60.0)
                    )
            ),

            //// SHORT BLUE
            dBackdropOffset = Pose(-0.0, -0.0, 0.0),

            //// SHORT BLUE
            xStack = TrajCoef(
                Pose(-81.0, 168.0, 1.57),
            ),


            //// SHORT BLUE
            putPark = TrajCoef(
                    Pose(-16.0, -100.0, 1.57),
                    Vec2d(60.0, 0.5), Vec2d(60.0, 2.7)
            )
    )

    @JvmField
    var shortRed = ShortVals(
            //// SHORT RED
            aStartPreload = Vec3T( /// Pos 0 = under thruss, 1 = mid, 2 e odar
                    TrajCoef(
                            Pose(-75.0, 12.0, 1.57),
                            Vec2d(10.0, 1.6), Vec2d(60.0, 1.5),
                            Vec2d(0.3, 0.6), 0.8
                    ),
                    TrajCoef(
                            Pose(-66.0, 10.0, 0.2),
                            Vec2d(), Vec2d(2.0, 0.0),
                            Vec2d(0.2, 0.4), 0.8,
                            Vec2d(34.0, 60.0)
                    ),
                    TrajCoef(
                            Pose(-40.0, 30.0, 0.0), Vec2d(0.2, 0.4), 1.0
                    )
            ),

            //// SHORT RED
            aPreloadBackdrop = TrajCoef(
                    Pose(0.0, 101.4, -1.57), Vec2d(), Vec2d(),
                    Vec2d(0.2, 0.4), 0.9, Vec2d(30.0, 60.0)
            ),

            //// SHORT RED
            backdropPosX = Vec3(-86.0, -70.0, -60.0),

            //// SHORT RED
            backdropStack = Vec4T(
                    TrajCoef(
                            Pose(-14.0, 10.0, -1.5),
                            Vec2d(30.0, -1.57), Vec2d(20.0, 1.64),
                            Vec2d(0.0, 0.2), 0.8
                    ),
                    TrajCoef(
                            Pose(-15.0, -90.0, -1.57),
                            Vec2d(5.0, 0.7), Vec2d(0.0, 0.0),
                            Vec2d(0.0, 0.1), 0.8
                    ),
                    TrajCoef(
                            Pose(-65.0, -168.0, -1.5),
                            Vec2d(40.0, -1.57), Vec2d(20.0, 1.4), 0.8,
                            Vec2d(20.0, 50.0)
                    ),
                    TrajCoef()
            ),

            //// SHORT RED
            bStackOffset = Pose(-2.0, -1.5, 0.0),

            //// SHORT RED
            cStackBackdrop = Vec3T(
                    TrajCoef(
                            Pose(-16.0, -120.0, -1.57),
                            Vec2d(15.0, 2.2), Vec2d(30.0, -1.6),
                            Vec2d(0.2, 0.4), 0.8
                    ),
                    TrajCoef(
                            Pose(-17.0, 45.0, -1.57),
                            Vec2d(0.0, 0.2), Vec2d(0.0, 0.0), 0.8,
                            Vec2d(0.0, 0.1)
                    ),
                    TrajCoef(
                            Pose(-66.0, 103.5, -1.57),
                            Vec2d(20.0, 1.5), Vec2d(40.0, -1.58),
                            1.0, Vec2d(50.0, 70.0)
                    )
            ),

            //// SHORT RED
            dBackdropOffset = Pose(-2.0, 0.0, 0.0),

            //// SHORT RED
            xStack = TrajCoef(
                    Pose(-110.0, -166.0, -1.57),
            ),

            //// SHORT RED
            putPark = TrajCoef(
                    Pose(-16.0, 100.0, -1.57),
                    Vec2d(80.0, -0.5), Vec2d(80.0, -2.7)
            )
    )
}

@Config
object AutoVars {
    @JvmField
    var NumCycles = 3

    @JvmField
    var WaitPut = 0.17

    @JvmField
    var WaitStack1 = 0.5

    @JvmField
    var WaitStack2 = 0.76

    @JvmField
    var WaitStack2Min = 0.5

    @JvmField
    var WaitStack3 = 0.1

    @JvmField
    var WaitPreload = 0.1

    @JvmField
    var INTAKEWAIT2 = 0.2

    @JvmField
    var INTAKEWAIT3 = 0.3

    @JvmField
    var GOUPDIST = 52.0

    @JvmField
    var STACKCORRTIME1 = 0.1

    @JvmField
    var STACKCORRTIME2 = 0.8

    @JvmField
    var KMS = 2.0

    @JvmField
    var SLEEPY_TIME_UWU = 3.3

    @JvmField
    var failsafe1 = Pose(0.0, -20.0, 0.0)

    @JvmField
    var failsafe1s = Pose(0.0, -2.0, 0.0)

    @JvmField
    var failsafe2 = Pose(10.0, 4.0, 0.0)

    val colours = arrayOf("#8EE3EF", "#AEF3E7", "#F6BD60", "#F7EDE2", "#37718E", "#F5CAC3", "#84A59D", "#F28482", "#19535F", "#0B7A75", "#D7C9AA", "#7B2D26", "#F0F3F5")
}
