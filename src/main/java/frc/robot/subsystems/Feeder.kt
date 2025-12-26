package frc.robot.subsystems

import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.DutyCycleOut
import com.ctre.phoenix6.controls.NeutralOut
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.ForwardLimitValue
import frc.robot.hid.MunchkinButtonBoard
import frc.robot.hid.MunchkinController

object Feeder {
    private val motor = TalonFX(16)

    init {
        val config =
            TalonFXConfiguration().apply {
                Feedback.SensorToMechanismRatio = 1.0
                Slot0.kP = 10.0
                ClosedLoopRamps.VoltageClosedLoopRampPeriod = 1.0
            }
        motor.configurator.apply(config)
    }

    enum class FeederState {
        Idle,
        HasNoteIdle,
        Intake,
        Shoot,
        Eject,
    }

    private val velocityControl = VelocityTorqueCurrentFOC(0.0)

    var currentState = FeederState.Idle

    val hasNote
        get() = motor.forwardLimit.value == ForwardLimitValue.ClosedToGround

    fun stateMachine() {
        when (currentState) {
            FeederState.Idle -> {
                motor.setControl(NeutralOut())
                if (MunchkinController.intake() || MunchkinButtonBoard.intake()) {
                    currentState = FeederState.Intake
                }
            }
            FeederState.HasNoteIdle -> {
                motor.setControl(NeutralOut())
                if (MunchkinController.shoot() || MunchkinButtonBoard.shoot()) {
                    currentState = FeederState.Shoot
                } else if (MunchkinController.eject() || MunchkinButtonBoard.eject()) {
                    currentState = FeederState.Eject
                }
            }
            FeederState.Intake -> {
                motor.setControl(
                    velocityControl.withVelocity(Intake.speed / 2.0).withIgnoreHardwareLimits(false)
                )
                if (hasNote) {
                    currentState = FeederState.HasNoteIdle
                } else if (MunchkinController.home() || MunchkinButtonBoard.home()) {
                    currentState = FeederState.Idle
                }
            }
            FeederState.Shoot -> {
                // Shooter wheels are 2.875 inches, Feeder wheel is 2 inches
                motor.setControl(
                    velocityControl
                        .withVelocity(Shooter.speed * 1.4375)
                        .withIgnoreHardwareLimits(true)
                )
                if (motor.forwardLimit.value == ForwardLimitValue.Open) {
                    currentState = FeederState.Idle
                } else if (MunchkinController.home() || MunchkinButtonBoard.home()) {
                    currentState =
                        if (hasNote) {
                            FeederState.HasNoteIdle
                        } else {
                            FeederState.Idle
                        }
                }
            }

            FeederState.Eject -> {
                motor.setControl(DutyCycleOut(-.5))
                if (MunchkinController.home() || MunchkinButtonBoard.home()) {
                    currentState = FeederState.Idle
                }
            }
        }
    }
}
