package frc.robot.commands

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.FeederSubsystem
import frc.robot.subsystems.IntakeSubsystem
import frc.robot.subsystems.WristSubsystem

/** Runs the intake and feeder until the note is detected by the beam break */
fun intakeNote(): Command {
    return WristSubsystem.raiseWrist()
        .until { WristSubsystem.atIntakePosition }
        .andThen(IntakeSubsystem.Intake().withDeadline(FeederSubsystem.Intake()))
}
