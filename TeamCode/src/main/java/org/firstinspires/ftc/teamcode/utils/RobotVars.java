package org.firstinspires.ftc.teamcode.utils;

import static java.lang.Math.PI;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.hardware.PIDFC;
import org.firstinspires.ftc.teamcode.hardware.Timmy;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Config
public class RobotVars {
    public static boolean USE_TELE = true;
    public static boolean USE_CAMERA = true;
    public static boolean USE_LOCALIZER = true;
    public static boolean USE_FIELD_CENTRIC = true;
    public static boolean USE_DIFFY = true;
    public static boolean USE_SENSORS = true;
    public static boolean USE_IMU_LOCALIZER = true;
    public static boolean USE_RIDICARE = true;
    public static boolean USE_SWERVE = true;
    public static boolean USE_INTAKE = true;
    public static boolean USE_SWERVE_MOTORS = true;
    public static boolean USE_AUTO_MOVE = true;

    public static Timmy timmy;
    public static boolean TimmyAddKILLLLLLLL = false;
    public static boolean TimmyToClose = true;
    public static double TimmyLoopTime = 0.5;
    public static double TimmyCurOff = 0.0;

    public static PIDFC RidicarePid = new PIDFC(0.0035, 0.0, 0.0, 0.0);
    public static double RidicareTime = 0.5;
    public static double RidicareLeeway = 0.5;
    public static double RidicareMaxTime = 0.1;
    public static double RidicarePower = -0.5;
    public static String RidicareEncoderName = "LFM";
    public static int RidicareEncoderDir = -1;
    public static int RidicareHaveIHangedMyself = 300;
    public static int RidicareIHaventHangedMyslef = 400;
    public static double RidicareHANGEDMYSELF = -0.65;
    public static double RidicareHANGINGMYSELF = -1.0;

    public static int RBOT_POS = -5;
    public static int RMID_POS = 300;
    public static int RTOP_POS = 1400;

    //public static Vec4 SwerveWheelOffsets = new Vec4(-2.4828562817478184, -2.237612437685438, -0.6482802389555943, -3.533453402390716);
    /*
    public static double OFFLF = -2.4828562817478184;
    public static double OFFLB = -2.237612437685438;
    public static double OFFRF = -0.6482802389555943;
    public static double OFFRB = -3.533453402390716;
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

    // LF RF RB LB
    public static Vec4 SwerveWheelOffsets = new Vec4(-2.4828562817478184, -0.6482802389555943, -3.533453402390716, -2.237612437685438);
    public static Vec4P SwervePids = new Vec4P(
            new PIDFC(0.55, 0.0, 0.0, 0.03),
            new PIDFC(0.6, 0.0, 0.0, 0.03),
            new PIDFC(0.65, 0.0, 0.0015, 0.015),
            new PIDFC(0.45, 0.0, 0.0, 0.00)
    );
    public static Vec4 SwerveStaticRotation = new Vec4(0.08, 0.08, 0.08, 0.08);
    public static double SwerveAngP = -0.38;
    public static double SwerveHeadP = 1.0;
    public static double SwerveMaxPower = 1.0;
    public static double SwerveKmsConf = 3;
    public static double SwerveManualTurnPower = 0.8;
    public static boolean SwerveCanInvertMotor = true;
    public static double SwerveTrackWidth = 21.0;
    public static double SwerveWheelBase = 21.0;
    public static boolean __SwerveMove = true;
    public static double SwerveAntiRetardationForce = -0.1;
    public static double SwerveAntiRetardationTime = 0.15;
    public static double SwerveTurnWaitTime = 0.2;
    public static double SwerveTurnMaxDif = 0.2;
    public static PIDFC SwerveTurnPIDC = new PIDFC(-0.5, 0.0, 0.0, 0.0);

    public static double DiffyLOff = 0;
    public static double DiffyROff = 0;

    public static boolean KILLMYSELF = false;
    public static double DiffyWaitUpTurn = 0.88;
    public static double DiffyWaitDownTurn = 0.98;
    public static double DiffyPreloadUp = 1.21;
    public static double DiffyUpSafe = 1.2;
    public static double DiffyAUp = -0.155; /// Unghi cu pixel sus
    public static double DiffyADown = 0.12; /// Unghi fara pixel jos
    public static double DiffyUp = 1.03; /// Pozitie cu pixel sus
    public static double DiffyDown = 0.5;
    public static double DiffyPrepDown = 0.637; /// Pozitie sta jos asteapta


    public static double DiffyMidUp = 0.72;
    public static double DiffyMid2Up = 0.85;
    public static double DiffyMidDown = 0.70;
    public static double DiffyEncROff = -1.5;
    public static double DiffyEncLOff = -2.4;
    public static double Diffy__UMVEL = 3;
    public static double Diffy__UMAC = 8;
    public static double Diffy__UMDC = 1.5;
    public static double Diffy__UMDCAuto = 0.1;
    public static double Diffy__UMDCOp = 1.5;

    public static double Diffy__MVEL = 4;
    public static double Diffy__MAC = 6;
    public static double Diffy__MDC = 1.3;

    public static double ClownFInchis = 0.692;
    public static double ClownFDeschis = 0.12;
    public static double ClownNInchis = 0.64;
    public static double ClownNDeschis = 0.20;
    public static double ClownWait1 = 0.1;
    public static double ClownWait2 = 0.1;
    public static double ClownWait3 = 0.2;
    public static double ClownWait4 = 0.15;
    public static double ClownWait5 = 0.15;
    public static double ClownWaitDown1 = 0.05;
    public static double ClownWaitDown2 = 0.08;
    public static double ClownWaitDown3 = 0.36;
    public static double ClownWaitDown4 = 0.20;
    public static double ClownWaitDown5 = 0.30;
    public static double ClownWaitDown6 = 0.10;

    public static int SensorsMinDist = 160;

    public static double ClownPWait1 = 0.1;
    public static double ClownPWait2 = 0.0;
    public static double ClownPWait3 = 0.3;
    public static double ClownPWaitDown1 = 0.19;

    public static double GelenkCenter = 0.48;
    public static double GelenkDif = 0.10;

    public static double IntakePower = -1.0;
    public static double IntakeRevPower = 0.8;
    public static Vec4 IntakeGet = new Vec4(0.38, 0.155, 0.4, 0.28); // Get 1 2 Up 1 2
    public static Vec4 IntakeGetUp = new Vec4(0.38, 0.2, 0.47, 0.45); // Get 1 2 Up 1 2
    public static Vec4 IntakeGetCostac = new Vec4(0.41, 0.17, 0.40, 0.14); // Get 1 2 Up 1 2
    public static Vec4 IntakeStack1 = new Vec4(0.41, 0.18, 0.43, 0.27); // AfterShave 1 2 Stack 1 2
    public static Vec4 IntakeStack2 = new Vec4(0.45, 0.19, 0.42, 0.25); // Prep 1 2 Stack 1 2
    public static Vec4 IntakeStack3 = new Vec4(0.38, 0.195, 0.36, 0.26); // Prep 1 2 Stack 1 2
    public static int __IntakeSetStatus = 20;

    public static boolean __LOG_STATUS = false;

    public static double SERVO_GEAR_RATIO = 1.000;

    public static double AvionInchis = 1.0;
    public static double AvionDeschis = 0.7;

    public static double WheelsAlignMin = 0.60;
    public static double WheelsAlignMax = 1.00;
    public static double WheelsAlignStart = PI / 2 - PI / 4;
    public static double WheelsAlignEnd = PI / 2 - PI / 8;

    public static Pose WheelsAdjPose = new Pose (-0.15, 0.2, 0.0);
    public static String WheelsPerpName = "LBM";
    public static Pose WheelsPerpPos = new Pose(-13.3, 1.15, PI / 2);
    public static Integer WheelsPerpDir = 1;
    public static String WheelsParRName = "RFM";
    public static Pose WheelsParRPos = new Pose(-8.15, -5.5, 0.0);
    public static Integer WheelsParRDir = 1;
    public static String WheelsParLName = "RBM";
    public static Pose WheelsParLPos = new Pose(-8.15, 4.2, 0.0);
    public static Integer WheelsParLDir = -1;
    public static Double WheelsTicksToCm = 1.8 * 2 * PI / 8192.0 / 1.02; // Radius * 2pi / Ticks/Rev / DracuStie

    public static String CameraName = "Anticamera";
    public static OpenCvCameraRotation CameraOrientation = OpenCvCameraRotation.UPSIDE_DOWN;
    public static int CameraGain = 60;
    public static int CameraExposure = 60;

    public static double EncoderPowerFuckery = 0.06;
    public static double EncoderAccelFuckery = 0.00;
    public static boolean OpModeKMS = false;
    public static boolean OpModeKMSShort = false;
    public static boolean __IsAuto = false;
    public static boolean __AutoShort = false;
    public static boolean __UPDATE_SENSORS = false;
    public static boolean __UPDATE_DIFFY = false;
    public static boolean __UPDATE_INTAKE = false;
    public static double ___CURRENT_SCHWERVE_SWPEED = 0.0;
    public static double ___CURRENT_SCHWERVE_ACCEL = 0.0;
    public static boolean ___KILL_DIFFY_THREADS = false;
    public static double __FUNNY_SWERVE_COEF = 1.1;

    public static Pose InfPos = new Pose(10000000000.0, 1000000000000.0, 0.0);

    /*
     * Expansion:
     *     Motors:
     *         0: RidL
     *         1: Intake
     *         2:
     *         3: RidR
     *     Servos:
     *         0: GhearaFar
     *         1: DifLS (5 sus in jos)
     *         2: Gelenk (3 sus in jos)
     *         3: DifRS (1 sus in jos)
     *         4: GhearaNear
     *         5: RidIntake (6 sus in jos)
     *      Analog:
     *         0-1: DifLSE DifRSE
     * Control:
     *     Motors:
     *         0: LBADM
     *         1: LFM
     *         2: RBM
     *         3: RFM
     *     Servos:
     *         0: RBS (3 dreapta stanga)
     *         1: Pewpew
     *         2: LFS
     *         3: LBS
     *         4: RFS
     *         5: RidIntakeFar
     *      Analog:
     *         0-1: LFE LBE
     *         2-3: RBE RFE
     *      i2c:
     *         2: SensorFar
     *         3: SensorNear
     */
}
