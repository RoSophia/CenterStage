package org.firstinspires.ftc.teamcode.auto

import com.acmerobotics.dashboard.config.Config
import org.firstinspires.ftc.teamcode.pp.PP.MAX_FRACTION
import org.firstinspires.ftc.teamcode.pp.TrajCoef
import org.firstinspires.ftc.teamcode.utils.Pose
import org.firstinspires.ftc.teamcode.utils.Vec2d

@Config
object BlueLongP {
    @JvmField
    var bPos2 = TrajCoef(
            Pose(-94.0, 10.0, 0.5),
            0.4
    )

    @JvmField
    var bPos1 = TrajCoef(
            Pose(-111.0, -23.0, 0.0),
            Vec2d(0.0, 2.0), Vec2d(0.0, 3.0),
            0.4
    )

    @JvmField
    var bPos0 = TrajCoef(
            Pose(-85.0, -25.0, -1.57),
            Vec2d(80.0, 1.8), Vec2d(40.0, 1.4),
            0.4
    )

    @JvmField
    var bPos2Stack = TrajCoef(
            Pose(-125.0, 10.2, 1.57),
            Vec2d(60.0, 4.4), Vec2d(0.0, 4.0),
            0.5
    )

    @JvmField
    var bPos1Stack = TrajCoef(
            Pose(-125.0, 10.0, 1.57),
            Vec2d(30.0, 3.1), Vec2d(20.0, 4.0),
            0.4
    )

    @JvmField
    var bPos0Stack = TrajCoef(
            Pose(-125.0, 10.0, 1.57),
            Vec2d(60.0, 1.7), Vec2d(0.0, 0.0),
            Vec2d(0.2, 0.7),
            0.5
    )

    @JvmField
    var bPutPos = TrajCoef(
            Pose(), Pose(-65.0, -245.0, 1.57),
            Vec2d(200.0, 4.65), Vec2d(80.0, 1.9),
            Vec2d(0.0, 1.0), 0.5, Vec2d(40.0, 80.0)
    )

    @JvmField
    var bPutPosCase = Pose(-25.0, -50.0, -70.0)

    @JvmField
    var bOffsets = Pose(0.0, 0.0, 0.0)

    @JvmField
    var bStackOffset = Pose(0.0, -0.0, 0.0)

    @JvmField
    var bStackPos2 = TrajCoef(
            bPutPos.ep,
            Pose(-130.0, 58.0, 1.57),
            Vec2d(100.0, -3.5), Vec2d(100.0, -1.35),
            Vec2d(), MAX_FRACTION, Vec2d(60.0, 100.0)
    )

    @JvmField
    var bparkPos = TrajCoef(
            Pose(-105.0, -237.0, 1.57),
            Vec2d(30.0, 1.7), Vec2d(30.0, 1.7)
    )
}

@Config
object BlueShortP {
    @JvmField
    var sbPos0 = TrajCoef(
            Pose(-80.0, 15.0, 0.5),
            Vec2d(50.0, -2.0), Vec2d(30.0, -1.6),
            0.45
    )

    @JvmField
    var sbPos1 = TrajCoef(
            Pose(-113.0, -16.0, 0.0),
            Vec2d(40.0, -2.0), Vec2d(15.0, -3.0),
            0.4
    )

    @JvmField
    var sbPos2 = TrajCoef(
            Pose(-75.0, -33.0, 1.57),
            0.5
    )

    @JvmField
    var sbPos0Stack = TrajCoef(
            Pose(-60.0, -45.0, 1.57),
            Vec2d(60.0, -1.7), Vec2d(0.0, 0.0),
            Vec2d(0.2, 0.7),
            0.5
    )

    @JvmField
    var sbPos1Stack = TrajCoef(
            Pose(-60.0, -45.0, 1.57),
            Vec2d(30.0, 3.1), Vec2d(40.0, 4.0),
            0.5
    )

    @JvmField
    var sbPos2Stack = TrajCoef(
            Pose(-65.0, -60.0, 1.57),
            0.5
    )

    @JvmField
    var sbPutPos = TrajCoef(
            Pose(), Pose(-65.0, -100.0, 1.57),
            Vec2d(0.0, 0.5), Vec2d(0.0, 1.9),
            Vec2d(0.0, 1.0), MAX_FRACTION, Vec2d(80.0, 140.0)
    )

    @JvmField
    var sbparkPos = TrajCoef(
            Pose(-105.0, -90.0, 1.57),
            Vec2d(30.0, 1.7), Vec2d(30.0, 1.7)
    )
}

@Config
object RedLongP {
    @JvmField
    var rPos2 = TrajCoef(
            Pose(-92.0, 24.0, 0.5),
            0.4
    )

    @JvmField
    var rPos1 = TrajCoef(
            Pose(-120.0, -0.0, 0.0),
            Vec2d(0.0, 2.0), Vec2d(0.0, 3.0),
            0.4
    )

    @JvmField
    var rPos0 = TrajCoef(
            Pose(-85.0, -20.0, -1.57),
            Vec2d(80.0, 1.8), Vec2d(40.0, 1.4),
            0.4
    )

    @JvmField
    var rPos2Stack = TrajCoef(
            Pose(-125.0, 10.2, 1.57),
            Vec2d(60.0, 4.4), Vec2d(0.0, 4.0),
            0.5
    )

    @JvmField
    var rPos1Stack = TrajCoef(
            Pose(-125.0, 10.0, 1.57),
            Vec2d(30.0, 3.1), Vec2d(20.0, 4.0),
            0.4
    )

    @JvmField
    var rPos0Stack = TrajCoef(
            Pose(-125.0, 10.0, 1.57),
            Vec2d(60.0, 1.7), Vec2d(0.0, 0.0),
            Vec2d(0.2, 0.7),
            0.5
    )

    @JvmField
    var rPutPos = TrajCoef(
            Pose(), Pose(-65.0, -245.0, 1.57),
            Vec2d(200.0, 4.65), Vec2d(80.0, 1.9),
            Vec2d(0.0, 1.0), 0.5, Vec2d(40.0, 80.0)
    )

    @JvmField
    var rOffsets = Pose(1.0, 0.0, 2.5)

    @JvmField
    var rStackOffset = Pose(0.0, -0.0, 0.0)

    @JvmField
    var rStackPos2 = TrajCoef(
            rPutPos.ep,
            Pose(-130.0, 58.0, 1.57),
            Vec2d(100.0, -3.5), Vec2d(100.0, -1.35),
            Vec2d(), MAX_FRACTION, Vec2d(60.0, 100.0)
    )

    @JvmField
    var rPutPosCase = Pose(-25.0, -50.0, -70.0)

    @JvmField
    var rparkPos = TrajCoef(
            Pose(-105.0, -237.0, 1.57),
            Vec2d(30.0, 1.7), Vec2d(30.0, 1.7)
    )
}

@Config
object RedShortP {
    @JvmField
    var srPos0 = TrajCoef(
            Pose(-80.0, 15.0, 0.5),
            Vec2d(50.0, -2.0), Vec2d(30.0, -1.6),
            0.45
    )

    @JvmField
    var srPos1 = TrajCoef(
            Pose(-113.0, -16.0, 0.0),
            Vec2d(40.0, -2.0), Vec2d(15.0, -3.0),
            0.4
    )

    @JvmField
    var srPos2 = TrajCoef(
            Pose(-75.0, -33.0, 1.57),
            0.5
    )

    @JvmField
    var srPos0Stack = TrajCoef(
            Pose(-60.0, -45.0, 1.57),
            Vec2d(60.0, -1.7), Vec2d(0.0, 0.0),
            Vec2d(0.2, 0.7),
            0.5
    )

    @JvmField
    var srPos1Stack = TrajCoef(
            Pose(-60.0, -45.0, 1.57),
            Vec2d(30.0, 3.1), Vec2d(40.0, 4.0),
            0.5
    )

    @JvmField
    var srPos2Stack = TrajCoef(
            Pose(-65.0, -60.0, 1.57),
            0.5
    )

    @JvmField
    var srPutPos = TrajCoef(
            Pose(), Pose(-65.0, -100.0, 1.57),
            Vec2d(0.0, 0.5), Vec2d(0.0, 1.9),
            Vec2d(0.0, 1.0), MAX_FRACTION, Vec2d(80.0, 140.0)
    )

    @JvmField
    var sbparkPos = TrajCoef(
            Pose(-105.0, -90.0, 1.57),
            Vec2d(30.0, 1.7), Vec2d(30.0, 1.7)
    )
}


@Config
object AutoVars {
    @JvmField
    var NumCycles = 0

    @JvmField
    var WaitIntake = 0.3

    @JvmField
    var WaitStack = 0.4

    @JvmField
    var cmtime = 2.0

    @JvmField
    var mtime = 2.0

    @JvmField
    var SLEEPY_TIME = 4.0

    @JvmField
    var sputPosCase = Pose(-27.0, -56.0, -70.0)

    val colours = arrayOf("#254E70", "#37718E", "#8EE3EF", "#AEF3E7", "#F6BD60", "#F7EDE2", "#F5CAC3", "#84A59D", "#F28482", "#19535F", "#0B7A75", "#D7C9AA", "#7B2D26", "#F0F3F5")
}