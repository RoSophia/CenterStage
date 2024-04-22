package org.firstinspires.ftc.teamcode.auto

import org.firstinspires.ftc.teamcode.auto.AutoVars.GOUPDISTBLUE
import org.firstinspires.ftc.teamcode.auto.AutoVars.GOUPDISTRED
import org.firstinspires.ftc.teamcode.auto.AutoVars.INTAKEWAIT2
import org.firstinspires.ftc.teamcode.auto.AutoVars.INTAKEWAIT3
import org.firstinspires.ftc.teamcode.auto.AutoVars.KMS
import org.firstinspires.ftc.teamcode.auto.AutoVars.SLEEPY_TIME
import org.firstinspires.ftc.teamcode.auto.AutoVars.WaitPreload
import org.firstinspires.ftc.teamcode.auto.AutoVars.WaitPut
import org.firstinspires.ftc.teamcode.auto.AutoVars.WaitStack1
import org.firstinspires.ftc.teamcode.auto.AutoVars.WaitStack2
import org.firstinspires.ftc.teamcode.auto.AutoVars.WaitStack2Min
import org.firstinspires.ftc.teamcode.auto.AutoVars.failsafe1
import org.firstinspires.ftc.teamcode.auto.AutoVars.failsafe1s
import org.firstinspires.ftc.teamcode.auto.AutoVars.failsafe2
import org.firstinspires.ftc.teamcode.hardware.CameraControls.AutoRed
import org.firstinspires.ftc.teamcode.hardware.CameraControls.AutoResult
import org.firstinspires.ftc.teamcode.hardware.Intakes.SIntake
import org.firstinspires.ftc.teamcode.hardware.Intakes.SInvert
import org.firstinspires.ftc.teamcode.hardware.Intakes.SKeep
import org.firstinspires.ftc.teamcode.hardware.Intakes.SStack1
import org.firstinspires.ftc.teamcode.hardware.Intakes.SStack2
import org.firstinspires.ftc.teamcode.hardware.Intakes.SStack3
import org.firstinspires.ftc.teamcode.hardware.Intakes.SStack4
import org.firstinspires.ftc.teamcode.hardware.Intakes.SStack5
import org.firstinspires.ftc.teamcode.hardware.Intakes.SStack6
import org.firstinspires.ftc.teamcode.hardware.Intakes.SUp
import org.firstinspires.ftc.teamcode.hardware.Intakes.SUpulLuiCostacu
import org.firstinspires.ftc.teamcode.utils.Pose
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.clown
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.etime
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.intake
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.pp
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.slides
import org.firstinspires.ftc.teamcode.utils.RobotVars.ClownFDeschis
import org.firstinspires.ftc.teamcode.utils.RobotVars.DiffyAUp
import org.firstinspires.ftc.teamcode.utils.RobotVars.DiffyUp
import org.firstinspires.ftc.teamcode.utils.RobotVars.DiffyUpSafe
import org.firstinspires.ftc.teamcode.utils.RobotVars.GelenkCenter
import org.firstinspires.ftc.teamcode.utils.RobotVars.GelenkDif
import org.firstinspires.ftc.teamcode.utils.RobotVars.IntakeRevPower
import org.firstinspires.ftc.teamcode.utils.RobotVars.RBOT_POS
import org.firstinspires.ftc.teamcode.utils.RobotVars.RMID_POS
import org.firstinspires.ftc.teamcode.utils.RobotVars.__ShortCentre
import org.firstinspires.ftc.teamcode.utils.RobotVars.__UPDATE_SENSORS
import org.firstinspires.ftc.teamcode.utils.TrajectorySequence

/***
 * ⢰⣶⣶⣶⣶⣶⣶⣶⣶⣶⣶⣶⣶⣶⣶⣶⣶⣶⣶⣶⣶⣶⣶⣶⣶⣶⣶⣶⣶⣶⣶⣶⣶⣶⣶⣶⣶⣶⣶⣶⣶⣶⣶⣶⣶⣶⣶⣶⣶⣶⣶⣶⣶⣶⣶⣶⣶⣶⣶⣶⣶⣶⣶⣶⣶
 * ⢸⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿
 * ⢸⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿
 * ⢸⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿
 * ⢸⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿
 * ⢸⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿
 * ⢸⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿
 * ⢸⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡟⣿⠻⠙⠻⠙⠛⢿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿
 * ⢸⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡿⠋⠀⢸⠃⣿⢹⠀⠀⠀⠀⠀⠈⠧⡙⠻⣌⠻⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿
 * ⢸⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡿⢁⣿⣿⠏⢩⣿⠏⠀⠀⠀⠃⣠⣯⠟⠀⠀⠀⠀⠀⠀⠀⠙⢦⣌⣷⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿
 * ⢸⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⠃⠀⣾⣿⠏⠀⣿⠏⠀⠀⠀⠀⢸⠙⡅⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠈⠙⠛⢿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿
 * ⢸⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣷⡅⢸⣿⡿⠀⠀⢿⡄⠀⠀⠀⠀⠘⢧⣻⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢸⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿
 * ⢸⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⢿⣿⠀⣾⣿⠇⠀⠀⠘⣷⠀⠀⠀⠀⠀⢈⡿⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿
 * ⢸⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣷⡄⢿⠀⡏⢿⡄⣀⣀⣀⣿⣀⠀⠀⠀⢀⡼⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿
 * ⢸⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡏⣀⡙⢿⣿⣿⣿⣿⣿⣿⣿⣇⢹⣸⠀⣧⣼⡿⠛⠛⠛⣿⣿⣿⣶⣤⣎⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢸⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿
 * ⢸⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣳⡏⠉⠀⠈⠹⣿⣿⣿⣿⠟⢻⠈⣿⠀⡇⠘⣿⠀⠀⢰⣯⠀⠈⠙⠿⢿⣷⡤⠀⠀⠀⠀⠀⠀⢀⣠⣶⣶⣶⡶⠷⣆⣸⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿
 * ⢸⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡇⠀⠀⣠⣤⣿⣿⣿⡟⠀⡜⠀⡿⠀⡇⢀⣿⣾⣿⣿⣿⣭⣽⡲⣄⠸⠛⢧⣄⠀⠀⠀⠀⠴⠛⠛⠛⠉⠉⠀⢀⡽⣸⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿
 * ⢸⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡏⠁⣰⠟⠁⠀⣿⣿⣿⠁⢀⡇⢰⠃⠀⡇⣸⣿⢿⣏⠹⣿⣶⣏⡽⣿⢛⣴⣿⠟⠀⠀⠀⠀⣤⣞⣯⣭⣭⣥⣤⡜⢀⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿
 * ⢸⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡿⡆⣿⠀⠀⠀⠹⣿⣿⠀⣼⢀⡎⠀⠀⣿⠋⡗⠀⠉⠉⠛⡇⠙⠒⠒⠋⠀⠀⠀⠀⠀⠀⣰⠉⠳⣄⢿⣶⢟⡿⠓⣸⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿
 * ⢸⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣧⢳⡈⠿⣷⠦⠀⢻⣿⢠⠇⡜⠁⠀⣴⣯⢰⠃⠀⠀⠀⣼⠇⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣿⠀⠀⠈⠉⠛⠉⠀⠀⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿
 * ⢸⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣧⠳⠀⠀⠀⠀⣼⡟⣾⣰⠃⠀⠀⡏⣧⢸⠀⠀⠀⡼⠁⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠁⠀⠀⠀⠀⠀⠀⠀⢸⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿
 * ⢸⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣷⣄⠀⠀⢠⡿⠁⣿⠇⠀⠀⠀⣷⣿⣿⠀⠀⢸⡇⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢸⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿
 * ⢸⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⠐⠋⠀⠀⠈⠃⠀⠀⠀⣿⠃⣿⠀⠀⠀⢻⡀⠀⠀⠀⠀⠀⠴⠾⠋⠁⠀⣴⠖⠀⠀⠀⠀⠀⠀⠀⣸⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿
 * ⢸⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⠀⠀⠀⠀⠀⠀⠀⠀⢰⠃⢀⣿⡀⠀⠀⠈⢷⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢠⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿
 * ⢸⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡯⠀⢷⠀⠀⠀⠀⠀⠀⠸⣤⡞⠉⣇⠀⠀⠀⠘⡇⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿
 * ⢸⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡗⠀⠈⣆⠀⠀⠀⠀⠀⣰⢿⡀⠀⣿⠀⠀⠀⢴⡧⠤⢤⣶⣶⣾⣷⣶⣤⣴⣦⡀⠀⠀⠀⠀⠀⣼⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿
 * ⢸⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡋⠳⣄⡀⠘⣇⠀⠀⠀⢰⠃⠀⢳⢀⡿⠀⠀⢠⡟⠀⠀⠀⠀⣄⠀⠈⠉⠉⠉⠙⠛⠓⠀⠀⠀⢰⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿
 * ⢸⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣦⣌⠙⢦⣌⠳⣄⠀⢸⠀⠀⢈⡽⠁⠀⡰⠋⠀⠀⠀⠀⠀⠉⠳⣤⣤⣬⡴⠀⠀⠀⠀⠀⢠⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿
 * ⢸⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣦⣌⠻⢮⣳⣼⡀⢀⡜⠁⠀⡼⠁⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣰⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿
 * ⢸⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣷⣦⣉⠻⢽⣾⣄⡀⠀⢷⡀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣠⣾⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿
 * ⢸⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣦⣝⢧⡉⠙⠲⠷⣄⣀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⣴⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿
 * ⢸⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣇⢳⡀⠀⠀⠘⣆⡽⢋⣴⣶⣦⣤⣤⣤⣤⣶⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿
 * ⢸⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡇⣷⠀⠀⢀⡿⢁⣾⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿
 * ⢸⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⢸⠀⢀⡾⢀⣾⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣷⣶⣤⣍⣉⡛⠛⠿⠿⣿⣿⣿
 * ⢸⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡄⡇⣼⠃⣾⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣶⣶⣤⣬⣉
 * ⢸⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⠁⣷⡏⢀⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿
 */

object Cele10Traiectorii {
    private fun overteemShort() = etime.seconds() > 24.4
    private fun overteemLong() = etime.seconds() > 26.0
    private fun endfs() = clown.sensorReadout() == 3 || overteemShort()
    private fun endfl() = clown.sensorReadout() == 3 || overteemLong()

    private fun si(i: Int, j: Int) = i - 2 * j

    private fun longFirst(ts: TrajectorySequence, i: Int, v: LongVals): TrajectorySequence {
        ts.at(v.cBackdropStack[0].s(ts).so(v.stackOffset * i).cb().t
                .addActionE(0.0) { clown.goDown(); slides.setTarget(RBOT_POS) })
                .at(v.cBackdropStack[1].s(ts).so(v.stackOffset * i).ce().st(0.6).t
                        .addActionE(100.0) { intake.status = SStack6 }
                        .addActionE(30.0) { __UPDATE_SENSORS = true })
                .aa { intake.sets(4 - 2 * i) }
                .sl(WaitStack1)
                .aa { intake.sets(3 - 2 * i) }
        return ts.slc(WaitStack2, ::endfl, WaitStack2Min)
                .failsafeMove(if (i == 0) ({ intake.status = SStack2 }) else ({ intake.status = SIntake }), ::endfl, ts.lastS++, ts.lastS++, failsafe1, failsafe2)
                .aa { clown.catchPixel(); __UPDATE_SENSORS = false }
                .at(v.bStackBackdrop[0].s(ts).so(v.cBackdropOffset * i - Pose(10.0, 0.0, 0.0)).cb().t
                        .addActionT(0.6) { intake.status = SInvert }
                        .addActionE(40.0) { intake.status = SUpulLuiCostacu }
                        .addActionE(20.0) { clown.goUp(-2) })
                .at(v.bStackBackdrop[1].s(ts).so(v.cBackdropOffset * i).ce().t
                        .addActionE(60.0) { pp.nextQRH = if (AutoRed) -1.57 else 1.57; pp.checkQR = true }
                        .addActionE(20.0) { slides.setTarget(RMID_POS) })
                .aa { clown.open() }
                .sl(WaitPut)
    }

    private fun longSecond(ts: TrajectorySequence, i: Int, v: LongVals) =
            ts.at(v.xBackdropStack[0].s(ts).so(v.stackOffset * i).cb().t
                    .addActionE(0.0) { clown.goDown(); slides.setTarget(RBOT_POS) })
                    .at(v.xBackdropStack[1].s(ts).so(v.stackOffset * i).cc().t
                            .addActionE(100.0) { intake.status = SStack6 })
                    .at(v.xBackdropStack[2].s(ts).so(v.stackOffset * i).ce().st(0.4).t)
                    .aa { intake.status = SStack4; }
                    .sl(WaitStack2Min)
                    .aa { clown.catchPixel() }
                    .at(v.xStackBackdrop[0].s(ts).ss(v.cBackdropStack[2].ep - Pose(0.0, -10.0, 0.0)).so(v.cBackdropOffset * i).cb().t
                            .addActionT(0.6) { intake.status = SInvert })
                    .at(v.xStackBackdrop[1].s(ts).so(v.cBackdropOffset * i).cc().t
                            .addActionE(10.0) { intake.status = SUpulLuiCostacu }
                            .addActionE(0.0) { clown.goUp(-2) })
                    .at(v.xStackBackdrop[2].s(ts).so(v.cBackdropOffset * i).ce().t
                            .addActionE(20.0) { slides.setTarget(RMID_POS) })
                    .aa { clown.open() }
                    .sl(WaitPut)
                    .aa { clown.goDown(); slides.setTarget(RBOT_POS); clown.gelenk?.position = GelenkCenter }
                    .sl(100.0)
                    .st(1002)

    fun getCycleTrajLong(v: LongVals): TrajectorySequence {
        val ts = TrajectorySequence()
        ts
                .aa { intake.status = SKeep; clown.targetPos = DiffyUpSafe }
                .at(v.aStartPreload[AutoResult].st(0.6).t /// Sets timeout to 0.6
                        .addActionT(0.1) { clown.goPreloadUp() })
                .aa { clown.ghearaFar?.position = ClownFDeschis }
                .sl(WaitPreload)
                .aa { clown.goPreloadDown(); intake.status = SStack6; __UPDATE_SENSORS = true }
                .at(v.bPreloadStack[AutoResult].s(ts).t)
                .aa { intake.status = SStack5 }
                .slc(WaitStack2, ::endfl, WaitStack2Min)
                .failsafeMove({ intake.status = SStack4 }, ::endfl, 2, 5, failsafe1, failsafe2)
                .sl(SLEEPY_TIME)
                .aa { clown.catchPixel(); __UPDATE_SENSORS = false }
                .at(v.bStackBackdrop[0].s(ts).cb().t
                        .addActionT(0.6) { intake.status = SInvert }
                        .addActionE(25.0) { intake.status = SUpulLuiCostacu })
                .at(v.bStackBackdrop[1].s(ts).sx(v.cBackdropPosX[AutoResult]).ce().t
                        .addActionS(0.0) { clown.goUp((if (AutoResult == 0) -2 else 2) * if (AutoRed) -1 else 1) }
                        .addActionE(50.0) { pp.nextQRH = if (AutoRed) -1.57 else 1.57; pp.checkQR = true }
                        .addActionS(20.0) { slides.setTarget(RMID_POS / 3) })
                .aa { clown.open() }
                .sl(WaitPut)

        for (i in 0 until 3) {
            ts.gt { if (etime.seconds() < 23.2) ts.lastS else 1000 }
            ts.st(ts.lastS++)
            if (i <= 1) {
                longFirst(ts, i, v)
            } else {
                longSecond(ts, i, v)
            }
        }

        ts.st(1000)
        ts.at(v.zBackdropPark.ss(v.bStackBackdrop[1].ep).t
                .addActionS(0.0) { clown.open() }
                .addActionS(0.0) { clown.goDown(); slides.setTarget(RBOT_POS) })
                .aa { clown.gelenk?.position = GelenkCenter }
        ts.st(1002)
        return ts
    }

    private fun antilonggput(ts: TrajectorySequence, i: Int, v: LongVals) = ts
            .at(v.zzCStackBackdrop[0].s(ts).ss(v.zzDBackdropStack[2].ep - Pose(0.0, 15.0, 0.0)).so(v.cBackdropOffset * (i - 1)).cb().t
                    .addActionT(0.5) { intake.status = SUp }
                    .addActionT(0.8) { intake.status = SUpulLuiCostacu })
            .at(v.zzCStackBackdrop[1].s(ts).so(v.cBackdropOffset * (i - 1)).cc().t
                    .addActionE(10.0) { clown.goUp((if (AutoResult == 0) -2 else 2) * if (AutoRed) -1 else 1) })
            .at(if (i == 0) {
                v.zzCStackBackdrop[2].s(ts).sx(v.cBackdropPosX[AutoResult])
            } else {
                v.zzCStackBackdrop[2].s(ts).so(v.cBackdropOffset * (i - 1))
            }.ce().t
                    .addActionE(50.0) { pp.nextQRH = if (AutoRed) -1.57 else 1.57; pp.checkQR = true }
                    .addActionS(20.0) { slides.setTarget(RMID_POS / 3) })
            .aa { clown.open() }
            .sl(WaitPut)

    private fun atstackLong(ts: TrajectorySequence, i: Int) = ts
            .aa { intake.sets(si(4, i)) }
            .sl(WaitStack1)
            .aa { intake.sets(si(3, i)) }
            .slc(WaitStack2, ::endfs, WaitStack2Min)
            .failsafeMove({ intake.sets(si(2, i)) }, ::endfs, ts.lastS++, ts.lastS++, failsafe1s, failsafe2)

    private fun antilongget(ts: TrajectorySequence, i: Int, v: LongVals) = ts
            .at(v.zzDBackdropStack[0].s(ts).so(v.stackOffset * i).cb().t
                    .addActionS(50.0) { clown.goDown(); slides.setTarget(RBOT_POS) })
            .at(v.zzDBackdropStack[1].s(ts).so(v.stackOffset * i).cc().t).aa { intake.status = SIntake; }
            .at(v.zzDBackdropStack[2].s(ts).so(v.stackOffset * i).ce().t
                    .addActionE(80.0) { intake.status = SStack6; __UPDATE_SENSORS = true })
            .kms(::atstackLong, i - 1)
            .aa { clown.catchPixel(); __UPDATE_SENSORS = false }

    fun getCycleTrajLongFFFFFFFFFFFFFFFFF(v: LongVals): TrajectorySequence {
        val ts = TrajectorySequence()
        ts
                .aa { intake.status = SKeep; clown.targetPos = DiffyUpSafe }
                .at(v.zzAPreload[AutoResult].st(0.6).t.addActionT(0.1) { clown.goPreloadUp() })
                .aa { clown.ghearaFar?.position = ClownFDeschis }.sl(WaitPreload)
                .aa { clown.goPreloadDown(); intake.status = SStack6; __UPDATE_SENSORS = true }
                .at(v.zzBStack.s(ts).t)
                .aa { intake.status = SStack5 }
                .slc(WaitStack2, ::endfl, WaitStack2Min)
                .failsafeMove({ intake.status = SStack4 }, ::endfl, 2, 5, failsafe1, failsafe2)
                .aa { clown.catchPixel(); __UPDATE_SENSORS = false }
                .sl (SLEEPY_TIME)

        antilonggput(ts, 0, v)
        for (i in 1 until 4) {
            ts.gt { if (etime.seconds() < 23.2) ts.lastS else 1000 }
            ts.st(ts.lastS++)

            antilongget(ts, i, v)
            antilonggput(ts, i, v)
        }
        ts.st(1000)
        ts.at(v.zBackdropPark.ss(v.bStackBackdrop[1].ep).t
                .addActionS(0.0) { clown.open() }
                .addActionS(0.0) { clown.goDown(); slides.setTarget(RBOT_POS) })
                .aa { clown.gelenk?.position = GelenkCenter }
        ts.st(1002)
        return ts
    }

    private fun atstackShort(ts: TrajectorySequence, i: Int) = ts
            .aa { intake.sets(si(5, i)) }
            .sl(WaitStack1)
            .aa { intake.sets(si(4, i)) }
            .slc(WaitStack2, ::endfs, WaitStack2Min)
            .failsafeMove({ intake.sets(si(3, i)) }, ::endfs, ts.lastS++, ts.lastS++, failsafe1s, failsafe2)

    private fun shortFirst(ts: TrajectorySequence, i: Int, v: ShortVals) = ts
            .at(v.backdropStack[0].s(ts).so(v.bStackOffset * i).cb().t
                    .addActionS(50.0) { clown.goDown(); slides.setTarget(RBOT_POS) })
            .at(v.backdropStack[1].s(ts).so(v.bStackOffset * i).cc().t)
            .aa { intake.status = SIntake; }
            .at(v.backdropStack[2].s(ts).so(v.bStackOffset * i)
                    .ce().t
                    .addActionE(80.0) { intake.status = SStack6; __UPDATE_SENSORS = true }) /// Set sp (with offset from last ep carried over) ; Add offset to ep; Set "Continue Continue"; Get traj
            .kms(::atstackShort, i)

    private fun shortFirstCenter(ts: TrajectorySequence, i: Int, v: ShortVals) = ts
            .at(v.zMidStack.s(ts).sex(v.zMidStack.ep.x).so(v.bStackOffset * i).t
                    .addActionS(50.0) { clown.goDown(); slides.setTarget(RBOT_POS) }
                    .addActionE(120.0) { intake.status = SStack6; __UPDATE_SENSORS = true })
            .kms(::atstackShort, i)

    private fun shortSecond(ts: TrajectorySequence, i: Int, v: ShortVals) = ts
            .at(v.backdropStack[0].s(ts).so(v.bStackOffset * i).cb().t
                    .addActionS(50.0) { clown.goDown(); slides.setTarget(RBOT_POS) })
            .at(v.backdropStack[1].s(ts).so(v.bStackOffset * i).cc().t)
            .aa { intake.status = SIntake; }
            .at(v.backdropStack[3].s(ts).so(v.bStackOffset * i).ce().t
                    .addActionE(150.0) { intake.status = SStack6 }
                    .addActionE(100.0) { intake.status = SIntake }
                    .addActionE(0.0) { intake.status = SStack6; __UPDATE_SENSORS = true })
            .kms(::atstackShort, i - 2)

    private fun shortSecondCenter(ts: TrajectorySequence, i: Int, v: ShortVals) = ts
            .at(v.zMidStack2.s(ts).so(v.bStackOffset * i).t
                    .addActionS(50.0) { clown.goDown(); slides.setTarget(RBOT_POS) }
                    .addActionE(110.0) { intake.status = SStack6; __UPDATE_SENSORS = true })
            .kms(::atstackShort, i - 2)

    private fun returnShort(ts: TrajectorySequence, i: Int, v: ShortVals) = ts
            .at(v.cStackBackdrop[0].s(ts).so(v.dBackdropOffset * i).cb().t
                    .addActionT(INTAKEWAIT2) { intake.intake.power = IntakeRevPower }
                    .addActionT(INTAKEWAIT3) { intake.status = SUp; __UPDATE_SENSORS = false })
            .aa { intake.status = SIntake; clown.open() }
            .at(v.cStackBackdrop[1].s(ts).so(v.dBackdropOffset * i).cc().t
                    .addActionS(20.0) { intake.status = SIntake }
                    .addActionS(30.0) { clown.catchPixel() }
                    .addActionE((if (AutoRed) GOUPDISTRED else GOUPDISTBLUE) + 20) { intake.status = SUpulLuiCostacu }
                    .addActionE(if (AutoRed) GOUPDISTRED else GOUPDISTBLUE) { clown.goUp(-1); })
            .at(v.cStackBackdrop[2].s(ts).so(v.dBackdropOffset * i).ce().t
                    .addActionE(80.0) { pp.nextQRH = if (AutoRed) -1.57 else 1.57; pp.checkQR = true }
                    .addActionE(70.0) { slides.setTarget(RMID_POS) })

    private fun returnShortCenter(ts: TrajectorySequence, i: Int, v: ShortVals) = ts
            .at(if (i < 2) {
                v.zMidBackdrop.s(ts).so(v.dBackdropOffset * i).t
            } else {
                v.zMidBackdrop.s(ts).ss(v.zMidStack.ep + Pose(0.0, (if (AutoRed) -1 else 1) * 20.0, 0.0)).so(v.dBackdropOffset * i).t
            }
                    .addActionT(INTAKEWAIT3) { intake.status = SUp; __UPDATE_SENSORS = false }
                    .addActionE(100.0) { intake.status = SUpulLuiCostacu }
                    .addActionE(80.0) { clown.goUp(-1); }
                    .addActionE(50.0) { pp.checkQR = true }
                    .addActionE(30.0) { slides.setTarget(RMID_POS) })

    private fun shortPreload(ts: TrajectorySequence, v: ShortVals) = ts
            .aa { clown.targetPos = DiffyUpSafe }
            .at(v.aStartPreload[AutoResult].t
                    .addActionT(0.1) { clown.goPreloadUp() }
                    .addActionS(60.0) { intake.status = SUpulLuiCostacu })
            .aa { clown.ghearaFar?.position = ClownFDeschis }
            .aa { clown.targetPos = DiffyUpSafe }
            .sl(0.1)
            .at(v.aPreloadBackdrop.s(ts).sx(v.backdropPosX[AutoResult]).t
                    .addActionE(60.0) {
                        clown.targetPos = DiffyUp
                        clown.targetAngle = DiffyAUp
                        clown.curState = 0
                        clown.gelenk?.position = GelenkCenter + (if (AutoResult == 2) -2 else 2) * (if (AutoRed) -1 else 1) * GelenkDif
                    })
            .aa { clown.open() }
            .sl(0.1)

    fun getCycleTrajShort(v: ShortVals): TrajectorySequence {
        val ts = shortPreload(TrajectorySequence(), v)

        for (i in 0 until 3) {
            if (i == 1) {
                ts.sl(SLEEPY_TIME)
            }
            ts.gt { if (etime.seconds() < 24.0) ts.lastS else 1000 }
            ts.st(ts.lastS++)

            if (i < 2) {
                if (__ShortCentre && AutoResult == 1) {
                    shortFirstCenter(ts, i, v)
                } else {
                    shortFirst(ts, i, v)
                }
            } else {
                if (__ShortCentre && AutoResult == 1) {
                    shortSecondCenter(ts, i, v)
                } else {
                    shortSecond(ts, i, v)
                }
            }
                    .aa { clown.catchPixel() }

            if (__ShortCentre && AutoResult == 1) {
                returnShortCenter(ts, i, v)
            } else {
                returnShort(ts, i, v)
            }
                    .aa { clown.open() }
                    .sl(WaitPut)

        }
        ts.st(1000)
        ts.at(v.putPark.s(ts).t
                .addActionS(0.0) { clown.open() }
                .addActionS(0.0) { clown.goDown(); slides.setTarget(RBOT_POS) })
        return ts
    }

    private fun shortFirstFFF(ts: TrajectorySequence, i: Int, v: ShortVals) = ts
            .at(v.zzAntiBackdropStack[0].s(ts).cb().t
                    .addActionS(50.0) { clown.goDown(); slides.setTarget(RBOT_POS) })
            .at(v.zzAntiBackdropStack[1].s(ts).ce().t
                    .addActionE(80.0) { intake.status = SStack6; __UPDATE_SENSORS = true })
            .kms(::atstackShort, i)

    private fun shortSecondFFF(ts: TrajectorySequence, i: Int, v: ShortVals) = ts
            .at(v.zzAntiBackdropStack[0].s(ts).cb().t
                    .addActionS(50.0) { clown.goDown(); slides.setTarget(RBOT_POS) })
            .at(v.zzAntiBackdropStack[2].s(ts).cc().t)
            .at(v.zzAntiBackdropStack[3].s(ts).ce().t
                    .addActionE(80.0) { intake.status = SStack6; __UPDATE_SENSORS = true })
            .kms(::atstackShort, i - 2)

    private fun returnShortFFF(ts: TrajectorySequence, i: Int, v: ShortVals) = ts
            .at(v.zzAntiStackBackdrop[0].ss(v.zzAntiBackdropStack[1].ep +
                    Pose(0.0, (if (AutoRed) -1 else 1) * 10.0, 0.0)).s(ts).cb().t
                    .addActionT(INTAKEWAIT3) { intake.status = SUp; __UPDATE_SENSORS = false })
            .at(v.zzAntiStackBackdrop[1].s(ts).so(v.dBackdropOffset * i).ce().t
                    .addActionE(90.0) { intake.status = SUpulLuiCostacu }
                    .addActionE(80.0) { clown.goUp(-1); }
                    .addActionE(60.0) { pp.nextQRH = if (AutoRed) -1.57 else 1.57; pp.checkQR = true }
                    .addActionE(30.0) { slides.setTarget(RMID_POS) })

    fun getCycleFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFShort(v: ShortVals): TrajectorySequence {
        val ts = shortPreload(TrajectorySequence(), v)
        for (i in 0 until 3) {
            ts.gt { if (etime.seconds() < 22.5) ts.lastS else 1000 }
            ts.st(ts.lastS++)

            if (i < 2) {
                shortFirstFFF(ts, i, v)
            } else {
                shortSecondFFF(ts, i, v)
            }.aa { clown.catchPixel() }

            returnShortFFF(ts, i, v)
                    .aa { clown.open() }
                    .sl(WaitPut)

        }
        ts.st(1000)
        ts.at(v.putPark.s(ts).t
                .addActionS(0.0) { clown.open() }
                .addActionS(0.0) { clown.goDown(); slides.setTarget(RBOT_POS) })
        return ts
    }
}
