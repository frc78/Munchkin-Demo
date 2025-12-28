package frc.robot

import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.DriverStation
import frc.robot.lib.degrees
import frc.robot.lib.inches
import frc.robot.lib.meters
import frc.robot.subsystems.Drivetrain
import frc.robot.subsystems.Elevator
import frc.robot.subsystems.IntakeFeederShooter
import frc.robot.subsystems.Wrist
import frc.robot.subsystems.drivetrain.Telemetry
import org.littletonrobotics.junction.LoggedRobot
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.networktables.NT4Publisher

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
object Robot : LoggedRobot() {

    init {
        Drivetrain.registerTelemetry { Telemetry.telemeterize(it) }
        DriverStation.silenceJoystickConnectionWarning(true)
        Logger.addDataReceiver(NT4Publisher())
        Logger.start()
        Drivetrain
        IntakeFeederShooter
        Wrist
        Elevator
    }

    override fun teleopPeriodic() {
        Drivetrain.stateMachine()
        IntakeFeederShooter.stateMachine()
        Wrist.stateMachine()
        Elevator.stateMachine()
    }

    private val componentPoses =
        NetworkTableInstance.getDefault()
            .getStructArrayTopic<Pose3d>("ComponentPoses", Pose3d.struct)
            .publish()

    override fun simulationInit() {
        Drivetrain.simulationInit()
    }

    override fun simulationPeriodic() {
        Elevator.simulationPeriodic()
        Wrist.simulationPeriodic()
        componentPoses.set(
            arrayOf(
                Pose3d(
                    0.0,
                    0.0,
                    Elevator.heightMeters + 18.inches.meters,
                    Rotation3d(0.degrees, Wrist.angle, 0.degrees),
                )
            )
        )
    }
}
