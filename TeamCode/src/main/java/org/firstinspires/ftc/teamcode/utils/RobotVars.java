package org.firstinspires.ftc.teamcode.utils;

import static java.lang.Math.PI;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.hardware.PIDFC;
import org.firstinspires.ftc.teamcode.hardware.Timmy;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.Vector;

@Config
public class RobotVars {
    public static boolean USE_TELE = true;
    public static boolean USE_CAMERA = true;
    public static boolean USE_LOCALIZER = true;
    public static boolean USE_FIELD_CENTRIC = true;
    public static boolean USE_DIFFY = true;
    public static boolean USE_IMU_LOCALIZER = true;
    public static boolean USE_RIDICARE = true;
    public static boolean USE_SWERVE = true;
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

    public static int RBOT_POS = 1;
    public static int RMID_POS = 1;
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

    public static double PaiplainMinSat = 50.00;
    public static double PaiplainMinVal = 40.00;
    public static double PaiplainBloo = 2.09;
    public static double PaiplainMaxBloo = 1.00;

    // LF RF RB LB
    public static Vec4 SwerveWheelOffsets = new Vec4(-2.4828562817478184, -0.6482802389555943, -3.533453402390716, -2.237612437685438);
    public static Vec4P SwervePids = new Vec4P(
            new PIDFC(0.55, 0.0, 0.0, 0.03),
            new PIDFC(0.6, 0.0, 0.0, 0.03),
            new PIDFC(0.7, 0.0, 0.0015, 0.03),
            new PIDFC(0.55, 0.0, 0.0, 0.00)
    );
    public static Vec4 SwerveStaticRotation = new Vec4(0.08, 0.08, 0.08, 0.08);
    public static double SwerveAngP = -0.38;
    public static double SwerveHeadP = 1.0;
    public static double SwerveMaxPower = 1.0;
    public static double SwerveKmsConf = 5;
    public static boolean SwerveCanInvertMotor = true;
    public static double SwerveTrackWidth = 21.0;
    public static double SwerveWheelBase = 21.0;
    public static boolean __SwerveMove = true;
    public static double SwerveAntiRetardationForce = -0.1;
    public static double SwerveAntiRetardationTime = 0.15;

    public static double GhearaSDESCHIS = 0.6;
    public static double GhearaSINCHIS = 0.42;

    public static double IntakePower = -1.0;
    public static Vec4 IntakeGet = new Vec4(0.38, 0.11, 0.39, 0.3); // Get 1 2 Up 1 2
    public static Vec4 IntakeGetUp = new Vec4(0.38, 0.2, 0.47, 0.45); // Get 1 2 Up 1 2
    public static Vec4 IntakeStack1 = new Vec4(0.45, 0.24, 0.40, 0.27); // Prep 1 2 Stack 1 2
    public static Vec4 IntakeStack2 = new Vec4(0.45, 0.19, 0.38, 0.23); // Prep 1 2 Stack 1 2
    public static Vec4 IntakeStack3 = new Vec4(0.45, 0.235, 0.40, 0.27); // Prep 1 2 Stack 1 2
    public static int __IntakeSetStatus = 20;

    public static double DiffyUp = 1.05;
    public static double DiffyDown = 0.6;
    public static double DiffyfUp = 0.0;
    public static double DiffyfDown = 0.29;
    public static double DiffyLOff = 0;
    public static double DiffyROff = 0;

    public static boolean AutoRed = true;
    public static int AutoMinBlocksRed = 20;
    public static int AutoMinBlocksBlue = 25;
    public static int AutoResult = 1;

    public static boolean __LOG_STATUS = false;

    public static double SERVO_GEAR_RATIO = 1.000;

    public static double AvionInchis = 0.38;
    public static double AvionDeschis = 0.52;

    public static double WheelsAlignMin = 0.60;
    public static double WheelsAlignMax = 1.00;
    public static double WheelsAlignStart = PI / 2 - PI / 4;
    public static double WheelsAlignEnd = PI / 2 - PI / 8;

    public static String WheelsPerpName = "RBM";
    public static Pose WheelsPerpPos = new Pose(-13.5, 1.2, PI / 2);
    public static Integer WheelsPerpDir = 1;
    public static String WheelsParRName = "RFM";
    public static Pose WheelsParRPos = new Pose(-8.15, -4.35, 0.0);
    public static Integer WheelsParRDir = 1;
    public static String WheelsParLName = "LBM";
    public static Pose WheelsParLPos = new Pose(-8.15, 5.2, 0.0);
    public static Integer WheelsParLDir = -1;
    public static Double WheelsTicksToCm = 1.8 * 2 * PI / 8192.0; // Radius * 2pi / Ticks/Rev

    public static String CameraName = "Anticamera";
    public static OpenCvCameraRotation CameraOrientation = OpenCvCameraRotation.UPSIDE_DOWN;
    public static int CameraGain = 70;
    public static int CameraExposure = 80;

    public static double EncoderPowerFuckery = 0.06;
    public static double EncoderFUCKFUCKFUCK = -0.0;
    public static double ___CURRENT_SCHWERVE_SWPEED = 0.0;
    public static Vector<Double> ___C = new Vector<>(4);
    public static int ___DELETE_THIS = 0;

    public static Pose InfPos = new Pose(10000000000.0, 1000000000000.0, 0.0);

    /*
     * Expansion:
     *     Motors:
     *         0: RidL
     *         1: Intake
     *         2:
     *         3: RidR
     *     Servos:
     *         0:
     *         1: DifRS (5 sus in jos)
     *         2: DifLS (3 sus in jos)
     *         3: Clown (1 sus in jos)
     *         4: SoftStage2VericuRupeAderenta
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
