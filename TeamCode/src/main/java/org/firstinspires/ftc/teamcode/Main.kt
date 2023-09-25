package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.controller
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.initma
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.preinit
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.startma
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.swerve
import kotlin.math.atan2
import kotlin.math.hypot

class Main : LinearOpMode() {
  override fun runOpMode() {
    preinit()
    initma(this)

    waitForStart()

    startma()

    while (isStopRequested) {
      controller.update()
      val speed = hypot(gamepad1.left_stick_x, gamepad1.left_stick_y).toDouble()
      val angle = atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4
      val turn = -gamepad1.right_stick_x.toDouble()

      swerve.move(speed, angle, turn)
    }
  }
}
