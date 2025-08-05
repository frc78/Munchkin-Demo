package frc.robot.subsystems

import com.ctre.phoenix6.controls.DutyCycleOut
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.ForwardLimitValue
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase

object FeederSubsystem : SubsystemBase() {
    private val motor = TalonFX(16)

    fun Intake(): Command {
        return startEnd(
                { motor.setControl(intakeControl.withOutput(0.5)) },
                { motor.setControl(intakeControl.withOutput(0.0)) },
            )
            .until { motor.forwardLimit.value == ForwardLimitValue.ClosedToGround }
    }

    fun shoot(): Command {
        return startEnd(
                { motor.setControl(shooterControl.withOutput(0.5)) },
                { motor.setControl(shooterControl.withOutput(0.0)) },
            )
            .until { motor.forwardLimit.value == ForwardLimitValue.Open }
    }

    fun outtake(): Command {
        return startEnd({ motor.setControl(outtakeControl) }, { motor.stopMotor() })
    }

    val intakeControl = DutyCycleOut(0.0).withIgnoreHardwareLimits(false)
    val outtakeControl = DutyCycleOut(-0.5).withIgnoreHardwareLimits(false)
    val shooterControl = DutyCycleOut(0.0).withIgnoreHardwareLimits(true)
}
