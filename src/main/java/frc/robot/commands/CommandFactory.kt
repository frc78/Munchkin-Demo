package frc.robot.commands

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.FeederSubsystem
import frc.robot.subsystems.IntakeSubsystem

/** Runs the intake and feeder until the note is detected by the beam break */
fun intakeNote(): Command {
    return IntakeSubsystem.Intake().withDeadline(FeederSubsystem.Intake())
}
