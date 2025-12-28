package frc.robot.subsystems

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.DutyCycleOut
import com.ctre.phoenix6.controls.Follower
import com.ctre.phoenix6.controls.NeutralOut
import com.ctre.phoenix6.controls.VelocityVoltage
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.ForwardLimitValue.ClosedToGround
import edu.wpi.first.units.measure.AngularVelocity
import frc.robot.hid.MunchkinButtonBoard
import frc.robot.hid.MunchkinController
import frc.robot.subsystems.IntakeFeederShooter.IFSState.Eject
import frc.robot.subsystems.IntakeFeederShooter.IFSState.Hold
import frc.robot.subsystems.IntakeFeederShooter.IFSState.Idle
import frc.robot.subsystems.IntakeFeederShooter.IFSState.Intake
import frc.robot.subsystems.IntakeFeederShooter.IFSState.Shoot
import frc.robot.subsystems.IntakeFeederShooter.IFSState.SpinUpFast
import frc.robot.subsystems.IntakeFeederShooter.IFSState.SpinUpMedium
import frc.robot.subsystems.IntakeFeederShooter.IFSState.SpinUpSlow
import org.littletonrobotics.junction.Logger

object IntakeFeederShooter {

    private val intakeLeader = TalonFX(9)
    private val intakeFollower = TalonFX(10)

    init {
        intakeLeader.configurator.apply(
            ClosedLoopRampsConfigs().withVoltageClosedLoopRampPeriod(1.0)
        )
        intakeFollower.setControl(Follower(intakeLeader.deviceID, false))
    }

    private val feederMotor = TalonFX(16)

    init {
        val feederConfig =
            TalonFXConfiguration().apply {
                Feedback.SensorToMechanismRatio = 1.0
                Slot0.kP = 10.0
                ClosedLoopRamps.VoltageClosedLoopRampPeriod = 1.0
            }
        feederMotor.configurator.apply(feederConfig)
    }

    private val shooterLeader = TalonFX(14)
    private val shooterFollower = TalonFX(15)

    init {
        val config = TalonFXConfiguration().apply { Feedback.SensorToMechanismRatio = 2.0 }
        shooterLeader.configurator.apply(config)

        shooterFollower.setControl(Follower(shooterLeader.deviceID, false))
    }

    private val shooterSpeed: AngularVelocity
        get() = shooterLeader.velocity.value

    enum class IFSState {
        Idle,
        Intake,
        Hold,
        Eject,
        SpinUpSlow,
        SpinUpMedium,
        SpinUpFast,
        Shoot,
    }

    private var currentState = Idle
        set(value) {
            Logger.recordOutput("IFS/current_state", value.name)
            field = value
        }

    init {
        Logger.recordOutput("IFS/current_state", currentState.name)
    }

    private val idleControl = NeutralOut()
    private val velocityControl = VelocityVoltage(0.0)
    private val intakeControl = DutyCycleOut(0.4)
    private val ejectControl = DutyCycleOut(-0.4)
    private val shootSlowControl = DutyCycleOut(0.33)
    private val shootMediumControl = DutyCycleOut(0.66)
    private val shootFastControl = DutyCycleOut(1.0)

    private val intakeSpeed: AngularVelocity
        get() = intakeLeader.velocity.value

    fun stateMachine() {
        when (currentState) {
            Idle -> {
                intakeLeader.setControl(idleControl)
                feederMotor.setControl(idleControl)
                shooterLeader.setControl(idleControl)
                if (MunchkinController.intake() || MunchkinButtonBoard.intake()) {
                    currentState = Intake
                }
            }
            Intake -> {
                // Reset shooter to idle because we can transition from shooting to intake
                shooterLeader.setControl(idleControl)
                intakeLeader.setControl(intakeControl)
                feederMotor.setControl(
                    velocityControl.withVelocity(intakeSpeed / 2.0).withIgnoreHardwareLimits(false)
                )
                if (MunchkinController.home() || MunchkinButtonBoard.home()) {
                    currentState = Idle
                } else if (feederMotor.forwardLimit.value == ClosedToGround) {
                    currentState = Hold
                }
            }
            Hold -> {
                intakeLeader.setControl(idleControl)
                feederMotor.setControl(idleControl)
                if (MunchkinController.eject() || MunchkinButtonBoard.eject()) {
                    currentState = Eject
                } else if (MunchkinController.slowShot() || MunchkinButtonBoard.slowShot()) {
                    currentState = SpinUpSlow
                } else if (MunchkinController.mediumShot() || MunchkinButtonBoard.mediumShot()) {
                    currentState = SpinUpMedium
                } else if (MunchkinController.fastShot() || MunchkinButtonBoard.fastShot()) {
                    currentState = SpinUpFast
                }
            }
            Eject -> {
                feederMotor.setControl(ejectControl)
                if (MunchkinController.home() || MunchkinButtonBoard.home()) {
                    currentState = Idle
                } else if (MunchkinController.intake() || MunchkinButtonBoard.intake()) {
                    currentState = Intake
                }
            }
            SpinUpSlow -> {
                shooterLeader.setControl(shootSlowControl)
                handleShootingInputs()
            }
            SpinUpMedium -> {
                shooterLeader.setControl(shootMediumControl)
                handleShootingInputs()
            }
            SpinUpFast -> {
                shooterLeader.setControl(shootFastControl)
                handleShootingInputs()
            }
            Shoot -> {
                feederMotor.setControl(
                    velocityControl
                        .withVelocity(shooterSpeed * 1.4375)
                        .withIgnoreHardwareLimits(true)
                )
                if (MunchkinController.home() || MunchkinButtonBoard.home()) {
                    currentState = Idle
                } else if (MunchkinController.intake() || MunchkinButtonBoard.intake()) {
                    currentState = Intake
                }
            }
        }
    }

    /**
     * Handles input checks for shooting states. All shooting states can transition between each
     * other
     */
    private fun handleShootingInputs() {
        if (MunchkinController.slowShot() || MunchkinButtonBoard.slowShot()) {
            currentState = SpinUpSlow
        } else if (MunchkinController.mediumShot() || MunchkinButtonBoard.mediumShot()) {
            currentState = SpinUpMedium
        } else if (MunchkinController.fastShot() || MunchkinButtonBoard.fastShot()) {
            currentState = SpinUpFast
        } else if (MunchkinController.shoot() || MunchkinButtonBoard.shoot()) {
            currentState = Shoot
        } else if (MunchkinController.home() || MunchkinButtonBoard.home()) {
            currentState = Hold
        }
    }

    val hasNote
        get() = feederMotor.forwardLimit.value == ClosedToGround
}
