package frc.robot.subsystems

import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.Follower
import com.ctre.phoenix6.controls.NeutralOut
import com.ctre.phoenix6.controls.VelocityVoltage
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.MotorAlignmentValue
import edu.wpi.first.units.measure.AngularVelocity
import frc.robot.hid.OI
import frc.robot.lib.rpm

object Shooter {

    private val shooterLeader = TalonFX(14)
    private val shooterFollower = TalonFX(15)

    init {
        val config = TalonFXConfiguration().apply { Feedback.SensorToMechanismRatio = 2.0 }
        shooterLeader.configurator.apply(config)

        shooterFollower.setControl(Follower(shooterLeader.deviceID, MotorAlignmentValue.Aligned))
    }

    private val shooterSpeed: AngularVelocity
        get() = shooterLeader.velocity.value

    val atShootingSpeed: Boolean
        get() = shooterSpeed > 400.0.rpm

    sealed class ShooterState {
        object Idle : ShooterState()

        class FixedSpeed(val speed: AngularVelocity) : ShooterState()
    }

    private var state: ShooterState = ShooterState.Idle

    private val idleControl = NeutralOut()
    private val velocityControl = VelocityVoltage(0.0)

    fun stateActions() {
        when (val state = state) {
            is ShooterState.Idle -> {
                shooterLeader.setControl(idleControl)
            }
            is ShooterState.FixedSpeed -> {
                shooterLeader.setControl(velocityControl.withVelocity(state.speed))
            }
        }
    }

    fun stateTransitions() {
        when (state) {
            is ShooterState.Idle -> {
                if (OI.spinUp()) {
                    state = ShooterState.FixedSpeed(500.0.rpm)
                }
            }
            is ShooterState.FixedSpeed -> {
                if (OI.home()) {
                    state = ShooterState.Idle
                }
            }
        }
    }
}
