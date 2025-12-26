package frc.robot.hid

import edu.wpi.first.wpilibj.GenericHID

object MunchkinButtonBoard {
    private val controller = GenericHID(2)

    fun raiseWrist() = controller.getRawAxis(1) > 0.5

    fun lowerWrist() = controller.getRawAxis(1) < -0.5

    fun slowShot() = controller.getRawButton(6)

    fun mediumShot() = controller.getRawButton(8)

    fun fastShot() = controller.getRawButton(10)

    fun shoot() = controller.getRawButton(1)

    fun raiseElevator() = controller.getRawButton(3)

    fun lowerElevator() = controller.getRawButton(4)

    fun intake() = controller.getRawButton(9)

    fun eject() = controller.getRawButton(7)

    fun home() = controller.getRawButton(5)
}
