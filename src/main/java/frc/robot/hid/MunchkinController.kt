package frc.robot.hid

import edu.wpi.first.math.MathUtil
import edu.wpi.first.wpilibj.XboxController
import kotlin.math.abs

object MunchkinController {
    private const val MAX_SPEED_MS = 3.0
    private const val MAX_ANGULAR_SPEED_RAD_S = 6.28

    val controller = XboxController(0)

    fun home() = controller.xButton

    fun raiseElevator() = controller.yButton

    fun intake() = controller.aButton

    fun eject() = controller.bButton

    fun raiseWrist() = controller.rightBumperButton

    fun lowerWrist() = controller.leftBumperButton

    fun slowShot() = controller.leftTriggerAxis in 0.1..<0.5

    fun mediumShot() = controller.leftTriggerAxis in 0.5..0.9

    fun fastShot() = controller.leftTriggerAxis >= 0.9

    fun shoot() = controller.rightTriggerAxis > 0.1

    fun resetRotation() = controller.startButton

    val translationX: Double
        get() {
            val x = MathUtil.applyDeadband(-controller.leftY, 0.1)
            return x * abs(x) * MAX_SPEED_MS
        }

    val translationY: Double
        get() {
            val y = MathUtil.applyDeadband(-controller.leftX, 0.1)
            return y * abs(y) * MAX_SPEED_MS
        }

    val rotation: Double
        get() {
            val r = MathUtil.applyDeadband(-controller.rightX, 0.1)
            return r * abs(r) * MAX_ANGULAR_SPEED_RAD_S
        }
}
