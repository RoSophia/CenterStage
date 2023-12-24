package org.firstinspires.ftc.teamcode.utils;

import com.acmerobotics.dashboard.config.Config;

import java.util.Dictionary;
import java.util.Map;

@Config
public class RobotVars {
    public static boolean USE_TELE = false; // Use Telemetry

    public static double pcoef = 1.0; // Power coefficient

    public static int RBOT_POS = 5;
    public static int RMIU_POS = 5;
    public static int RMID_POS = 5;
    public static int RTOP_POS = 5;

    public static String LES = "RF";
    public static String RES = "RB";
    public static String FES = "LB";
    public static boolean LER = true;
    public static boolean RER = true;
    public static boolean FER = true;

    public static double OFFLF = -0.4752787675627523;
    public static double OFFLB = -2.7166934353886925;
    public static double OFFRF = -0.08555017816129543;
    public static double OFFRB = -0.4809821127735054;
    //public static double OFFDL = 0.0;
    //public static double OFFDR = 0.0;

    public static double IND = 0.2;
    public static double INU = 0.6;

    public static double DIFLOFF = 0.0;
    public static double DIFUP = 0.3;
    public static double DIFDOWN = 0.7;
    public static double DIFFUP = 0.0;
    public static double DIFFDOWN = 0.4;

    public static double FUNKYLU = 0.01;
    public static double FUNKYLD = 0.84;
    public static double FUNKYRU = 0.55;
    public static double FUNKYRD = 0.0;

    public static double KMSCONF = 5;

    public static boolean canInvertMotor = true;
    public static boolean MOVE_SWERVE = true;
    public static double HEADP = 10.0;

    public static boolean LOG_STATUS = false;

    public static double SERVO_GEAR_RATIO = 1.021;

    public static double TRACK_WIDTH = 21.0;
    public static double WHEEL_BASE = 21.0;

    public static Map<String, Integer> nrRots;


    /*
     * Expansion:
     *     Motors:
     *         0: RidL
     *         1: Intake
     *         2: RidR
     *         3:   
     *     Servos:
     *         0: FunkyL
     *         1: DifLS (4 sus in jos)
     *         2: DifRS (3 sus in jos)
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
     *         3:       
     *         4: LBS
     *         5: RFS
      *      Analog:
      *         0-1: LFE LBE
      *         2-3: RBE RFE
      */
}
