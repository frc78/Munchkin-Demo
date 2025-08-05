package frc.robot

import com.ctre.phoenix6.swerve.SwerveModule
import com.ctre.phoenix6.swerve.SwerveRequest
import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj2.command.CommandScheduler
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import frc.robot.commands.intakeNote
import frc.robot.subsystems.Drivetrain
import frc.robot.subsystems.Elevator
import frc.robot.subsystems.FeederSubsystem
import frc.robot.subsystems.ShooterSubsystem
import frc.robot.subsystems.WristSubsystem

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

    private const val MAX_SPEED_MS = 1.0
    private const val MAX_ANGULAR_SPEED_RAD_S = 2.0

    val masterController = CommandXboxController(0)
    val demoController = CommandXboxController(1)

    val buttonBoard = CommandGenericHID(2)

    init {
        masterController.a().whileTrue(intakeNote())

        val drive =
            SwerveRequest.FieldCentric()
                .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage)
                .withDeadband(0.1)
                .withRotationalDeadband(0.1)
        Drivetrain.defaultCommand =
            Drivetrain.applyRequest {
                drive
                    .withVelocityX(-masterController.leftY * MAX_SPEED_MS)
                    .withVelocityY(-masterController.leftX * MAX_SPEED_MS)
                    .withRotationalRate(-masterController.rightX * MAX_ANGULAR_SPEED_RAD_S)
            }

        masterController.start().onTrue(Drivetrain.runOnce { Drivetrain.seedFieldCentric() })

        buttonBoard.axisGreaterThan(1, 0.5).whileTrue(WristSubsystem.lowerWrist())
        buttonBoard.axisLessThan(1, -0.5).whileTrue(WristSubsystem.raiseWrist())

        buttonBoard.button(6).whileTrue(ShooterSubsystem.shootSlow())
        buttonBoard.button(8).whileTrue(ShooterSubsystem.shootMedium())
        buttonBoard.button(10).whileTrue(ShooterSubsystem.shootFast())
        buttonBoard.button(1).onTrue(FeederSubsystem.shoot())

        buttonBoard.button(3).onTrue(Elevator.elevatorUp())
        buttonBoard.button(4).onTrue(Elevator.elevatorDown())
        buttonBoard.button(9).whileTrue(intakeNote())
        buttonBoard.button(7).whileTrue(FeederSubsystem.outtake())

        masterController
            .x()
            .whileTrue(
                Drivetrain.applyRequest {
                    drive
                        .withVelocityX(-demoController.leftY * MAX_SPEED_MS)
                        .withVelocityY(-demoController.leftX * MAX_SPEED_MS)
                        .withRotationalRate(-demoController.rightX * MAX_ANGULAR_SPEED_RAD_S)
                }
            )
    }

    override fun robotPeriodic() {
        CommandScheduler.getInstance().run()
    }
}
