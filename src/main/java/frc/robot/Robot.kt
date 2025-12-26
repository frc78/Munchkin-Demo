package frc.robot

import edu.wpi.first.wpilibj.TimedRobot
import frc.robot.subsystems.Drivetrain
import frc.robot.subsystems.Elevator
import frc.robot.subsystems.Feeder
import frc.robot.subsystems.Intake
import frc.robot.subsystems.Shooter
import frc.robot.subsystems.Wrist

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

    override fun teleopPeriodic() {
        Drivetrain.stateMachine()
        Feeder.stateMachine()
        Intake.stateMachine()
        Shooter.stateMachine()
        Wrist.stateMachine()
        Elevator.stateMachine()
    }
}
