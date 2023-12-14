package org.firstinspires.ftc.teamcode.utils;

import com.acmerobotics.dashboard.config.Config;

@Config
public class RobotVars {
    public static boolean USE_TELE = true; // Use Telemetry

    public static double pcoef = 1.0; // Power coefficient

    public static String LES = "RF";
    public static String RES = "RB";
    public static String FES = "LB";
    public static boolean LER = true;
    public static boolean RER = true;
    public static boolean FER = true;

    public static double OFFLF = -6.003721391852687;
    public static double OFFLB = -6.171019518034776;
    public static double OFFRF = -3.378281479836044;
    public static double OFFRB = -2.758517966934215;


    public static double KMSCONF = 5;


    public static boolean canInvertMotor = true;
    public static boolean MOVE_SWERVE = true;

    public static boolean LOG_STATUS = false;

    public static double SERVO_GEAR_RATIO = 1.0;

     /*
     * Expansion:
     *     Motors:
     *         0:   
     *         1:   
     *         2:   
     *         3:   
     *     Servos:
     *         0:         
     *         1:
     *         2:        
     *         3:
     *         4:       
     *         5:
     *      Analog:
     *         0-1:        
     * Control:
     *     Motors:
     *         0: LFM
     *         1: LBM
     *         2: RFM
     *         3: RBM
     *     Servos:
     *         0: LBS
     *         1: LFS
     *         2:
     *         3:       
     *         4: RFS
     *         5: RBS
      *      Analog:
      *         0-1: RBE RFE
      *         2-3: LFE LBE
      */
}
