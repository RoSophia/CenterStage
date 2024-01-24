package org.firstinspires.ftc.teamcode.utils;

import static org.firstinspires.ftc.teamcode.hardware.Intakes.SPStack1;
import static org.firstinspires.ftc.teamcode.hardware.Intakes.SStack1;
import static java.lang.Math.PI;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.hardware.PIDFC;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Config
public class RobotVars {
    public static boolean USE_TELE = true;
    public static boolean USE_CAMERA = false;
    public static boolean USE_LOCALIZER = true;
    public static boolean USE_FIELD_CENTRIC = true;
    public static boolean USE_DIFFY = true;
    public static boolean USE_RIDICARE = false;
    public static boolean USE_DIRECTION_PID = false;
    public static boolean MOVE_SWERVE = true;
    public static boolean MOVE_SWERVE_MOTORS = true;

    public static double TIMMYA = 0.5;
    public static double TIMMYOFF = 0.0;
    public static PIDFC RidicarePid = new PIDFC(0.0035, 0.0, 0.0, 0.0, 0.2);
    public static double RidicareTime = 0.5;
    public static double RidicareLeeway = 0.5;
    public static double RidicareMaxTime = 0.1;
    public static double RidicarePower = -0.3;
    public static String RidicareEncoderName = "LFM";
    public static int RidicareEncoderDir = -1;

    public static int RBOT_POS = 1;
    public static int RMID_POS = 5;
    public static int RTOP_POS = 1300;

    public static double OFFLF = -2.4828562817478184;
    public static double OFFLB = -2.237612437685438;
    public static double OFFRF = -0.6482802389555943;
    public static double OFFRB = -3.533453402390716;

    /*
    public static PIDFC WheelPidLFF = new PIDFC(0.34, 0.0, 0.004, 0.03);
    public static PIDFC WheelPidLBF = new PIDFC(0.34, 0.0, 0.004, 0.03);
    public static PIDFC WheelPidRFF = new PIDFC(0.34, 0.0, 0.004, 0.03);
    public static PIDFC WheelPidRBF = new PIDFC(0.34, 0.0, 0.004, 0.03);
     */
    /*
    public static PIDFC WheelPidLFF = new PIDFC(0.3, 0.0, 0.001, 0.03);
    public static PIDFC WheelPidLBF = new PIDFC(0.3, 0.0, 0.001, 0.03);
    public static PIDFC WheelPidRFF = new PIDFC(0.3, 0.0, 0.001, 0.03);
    public static PIDFC WheelPidRBF = new PIDFC(0.3, 0.0, 0.001, 0.03);
    public static PIDFC WheelPidLFF = new PIDFC(0.4, 0.0, 0.003, 0.0, 0.02);
    public static PIDFC WheelPidLBF = new PIDFC(0.4, 0.0, 0.003, 0.0, 0.02);
    public static PIDFC WheelPidRFF = new PIDFC(0.4, 0.0, 0.003, 0.0, 0.02);
    public static PIDFC WheelPidRBF = new PIDFC(0.4, 0.0, 0.003, 0.0, 0.02);
    */

    public static double PaiplainMinSat = 100.00;
    public static double PaiplainMinVel = 100.00;
    public static double PaiplainRed = 0.00;
    public static double PaiplainBloo = 2.09;
    public static double PaiplainMaxRed = 1.00;
    public static double PaiplainMaxBloo = 1.00;

    public static int __STATUS = 20;

    public static double SwerveHeadPidP = 1.0;
    public static double SwerveHeadPidI = 0.0;
    public static double SwerveHeadPidD = 0.0;
    public static double SwerveHeadPidF = 0.0;
    public static double SwerveMaxPower = 1.0;

    public static double GhearaSDESCHIS = 0.6;
    public static double GhearaSINCHIS = 0.47;

    public static double IntakePower = -0.7;
    public static double IntakePowerStack = -1.0;
    public static double IntakePrepDif = 0.2;
    public static double IntakePStack1 = 0.504;
    public static double IntakePStack2 = 0.46;
    public static double IntakePStack3 = 0.418;
    public static double IntakePDown = 0.42;
    public static double IntakePUp = 0.45;
    public static double IntakeWaitTime = 20.1;
    public static double IntakeWaitFallTime = 0.1;

    public static double SwerveMaxKeepAngTime = 0.1;

    public static double DLOFF = 0;
    public static double DROFF = 0;

    public static double DiffyUp = 1.05;
    public static double DiffyDown = 0.6;
    public static double DiffyfUp = 0.0;
    public static double DiffyfDown = 0.29;

    public static double KMSCONF = 5;

    public static boolean canInvertMotor = true;
    public static boolean _MOVE_SWERVE = true;
    public static double HEADP = 1.0;

    public static boolean AutoRed = true;
    public static int AutoMinBlocksRed = 10;
    public static int AutoMinBlocksBlue = 20;
    public static int AutoResult = 0;

    public static int SINTNIITNT = SPStack1;
    public static int SINTNIITNT2 = SStack1;

    public static double SwerveAngP = -0.38;

    public static boolean LOG_STATUS = false;

    public static double SERVO_GEAR_RATIO = 1.000;

    public static double TRACK_WIDTH = 21.0;
    public static double WHEEL_BASE = 21.0;

    public static double AvionInchis = 0.26;
    public static double AvionDeschis = 0.5;

    public static Pose LocalizerInitPos = new Pose(0.0, 0.0, 0.0);
    public static Pose AutoInitPos = new Pose(0.0, 0.0, 0.0);
    public static boolean AUTO_MOVE = true;

    public static double WheelsAlignMin = 0.60;
    public static double WheelsAlignMax = 1.00;
    public static double WheelsAlignStart = PI / 2 - PI / 4;
    public static double WheelsAlignEnd = PI / 2 - PI / 8;

    public static PIDFC WheelsPidF = new PIDFC(0.6, 0.0, 0.008, 0.0, 0.03);
    public static PIDFC WheelsPidB = new PIDFC(0.6, 0.0, 0.008, 0.0, 0.03);
    public static double WheelsMaxErr = 0.00;

    public static String WheelsPerpName = "RBM";
    public static Pose WheelsPerpPos = new Pose(-13, 1.8, PI / 2);
    public static Integer WheelsPerpDir = 1;
    public static String WheelsParRName = "RFM";
    public static Pose WheelsParRPos = new Pose(-8, -4.5, 0.0);
    public static Integer WheelsParRDir = 1;
    public static String WheelsParLName = "LBM";
    public static Pose WheelsParLPos = new Pose(-8, 5, 0.0);
    public static Integer WheelsParLDir = -1;
    public static Double WheelsTicksToCm = 1.8 * 2 * PI / 8192.0; // Radius * 2pi / Ticks/Rev

    public static Vec4 WheelsFBias = new Vec4(-0.02, -0.02, -0.02, -0.02);
    public static Vec4 WheelsBBias = WheelsFBias;
    public static Vec4 WheelsLBias = new Vec4(0.0, 0.0, 0.0, 0.0);
    public static Vec4 WheelsLatentVars = new Vec4(5.0, 5.0, 5.0, 5.0);

    public static String CameraName = "Anticamera";
    public static OpenCvCameraRotation CameraOrientation = OpenCvCameraRotation.UPSIDE_DOWN;
    public static int GAIN = 100;
    public static int EXPOSURE = 80;

    public static Pose InfPos = new Pose(10000000000.0, 1000000000000.0, 0.0);

    /*
     * Expansion:
     *     Motors:
     *         0: RidL
     *         1: Intake
     *         2: RidR
     *         3:
     *     Servos:
     *         0: FunkyL
     *         1: DifRS (5 sus in jos)
     *         2: DifLS (3 sus in jos)
     *         3: Clown (1 sus in jos)
     *         4: FunkyR
     *         5: RidIntake (6 sus in jos)
     *      Analog:
     *         0-1: DifLE DifRE
     * Control:
     *     Motors:
     *         0: LBM
     *         1: LFM
     *         2: RBM
     *         3: RFM
     *     Servos:
     *         0: RBS
     *         1:
     *         2: LFS
     *         3: LBS
     *         4: RFS
     *         5: Pewpew
     *      Analog:
     *         0-1: LFE LBE
     *         2-3: RBE RFE
     */
}