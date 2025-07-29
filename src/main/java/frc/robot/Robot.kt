package frc.robot

import com.ctre.phoenix6.swerve.SwerveModule
import com.ctre.phoenix6.swerve.SwerveRequest
import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import frc.robot.subsystems.Drivetrain
import frc.robot.subsystems.Elevator
import frc.robot.subsystems.IntakeSubsystem

/**
 * The functions in this object (which basically functions as a singleton class) are called
 * automatically corresponding to each mode, as described in the TimedRobot documentation. This is
 * written as an object rather than a class since there should only ever be a single instance, and
 * it cannot take any constructor arguments. This makes it a natural fit to be an object in Kotlin.
 *
 * If you change the name of this object or its package after creating this project, you must also
 * update the `Main.kt` file in the project. (If you use the IDE's Rename or Move refactorings when
 * renaming the object or package, it will get changed everywhere.)
 */
object Robot : TimedRobot() {

    private const val MAX_SPEED_MS = 2.0
    private const val MAX_ANGULAR_SPEED_RAD_S = 2.0
    private var autonomousCommand: Command? = null

    val controller = CommandXboxController(0)

    init {
        controller.a().whileTrue(IntakeSubsystem.Intake())

        val drive =
            SwerveRequest.FieldCentric()
                .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage)
                .withDeadband(0.1)
                .withRotationalDeadband(0.1)
        Drivetrain.defaultCommand =
            Drivetrain.applyRequest {
                drive
                    .withVelocityX(-controller.leftY * MAX_SPEED_MS)
                    .withVelocityY(-controller.leftX * MAX_SPEED_MS)
                    .withRotationalRate(-controller.rightX * MAX_ANGULAR_SPEED_RAD_S)
            }

        controller.start().onTrue(Drivetrain.runOnce { Drivetrain.seedFieldCentric() })
        controller.rightTrigger().onTrue(Elevator.elevatorUp())
        controller.leftTrigger().onTrue(Elevator.elevatorDown())
    }

    override fun robotPeriodic() {
        CommandScheduler.getInstance().run()
    }

    override fun teleopInit() {
        autonomousCommand?.cancel()
    }

    /** This method is called periodically during operator control. */
    override fun teleopPeriodic() {}

    override fun testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll()
    }
}
