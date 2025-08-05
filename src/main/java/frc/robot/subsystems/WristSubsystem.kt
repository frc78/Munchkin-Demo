package frc.robot.subsystems

import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.GravityTypeValue
import com.ctre.phoenix6.signals.InvertedValue
import edu.wpi.first.units.Units.Degrees
import edu.wpi.first.units.Units.Seconds
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase

object WristSubsystem : SubsystemBase() {
    val motor = TalonFX(13) // TODO Create a new TalonFX with id 13

    val MAX_ANGLE = Degrees.of(55.0)
    val MIN_ANGLE = Degrees.of(0.0)

    init {
        val config = TalonFXConfiguration()

        config.Feedback.SensorToMechanismRatio = 3 * 3 * 4 * 36 / 12.0
        config.SoftwareLimitSwitch.withReverseSoftLimitEnable(true)
            .withForwardSoftLimitEnable(true)
            .withReverseSoftLimitThreshold(Degrees.of(0.0))
            .withForwardSoftLimitThreshold(Degrees.of(55.0))
        config.Slot0.kP = 500.0
        config.Slot0.kV = 12.19
        config.Slot0.kA = 0.02
        config.Slot0.kG = 0.16
        config.Slot0.GravityType = GravityTypeValue.Arm_Cosine
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive
        config.ClosedLoopGeneral.ContinuousWrap = true

        motor.setPosition(MAX_ANGLE) // TODO set the correct wrist starting positon (55 degrees)
        motor.configurator.apply(config)

        defaultCommand =
            runOnce { motor.setControl(motorControl.withVelocity(0.0)) }.andThen(Commands.idle())
    }

    /** Speed at which the wrist should move when raising and lowering */
    val wristSpeed = (MAX_ANGLE - MIN_ANGLE) / Seconds.of(1.0)

    // TODO create a PositionVoltage control request
    val motorControl = PositionVoltage(MAX_ANGLE)

    fun raiseWrist(): Command {
        return this.run {
            var newPosition = motorControl.positionMeasure + wristSpeed * Seconds.of(.02)

            if (newPosition > MAX_ANGLE) {
                newPosition = MAX_ANGLE
            }

            motor.setControl(motorControl.withPosition(newPosition).withVelocity(wristSpeed))
        }
    }

    fun lowerWrist(): Command {
        return this.run {
            var newPosition = motorControl.positionMeasure - wristSpeed * Seconds.of(.02)

            if (newPosition < MIN_ANGLE) {
                newPosition = MIN_ANGLE
            }

            motor.setControl(motorControl.withPosition(newPosition).withVelocity(-wristSpeed))
        }
    }
}
