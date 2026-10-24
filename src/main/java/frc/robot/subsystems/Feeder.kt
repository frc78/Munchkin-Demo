package frc.robot.subsystems

import com.ctre.phoenix6.CANBus
import com.ctre.phoenix6.HootReplay.restart
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs
import com.ctre.phoenix6.configs.ProximityParamsConfigs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.ControlRequest
import com.ctre.phoenix6.controls.StaticBrake
import com.ctre.phoenix6.controls.VelocityVoltage
import com.ctre.phoenix6.hardware.CANrange
import com.ctre.phoenix6.hardware.TalonFX
import edu.wpi.first.wpilibj.Timer
import frc.robot.hid.OI
import frc.robot.lib.centimeters

object Feeder {

    private val feederMotor = TalonFX(16, CANBus("CANivore"))
    private val noteDetector =
        CANrange(1, CANBus("CANivore")).apply {
            configurator.apply(ProximityParamsConfigs().withProximityThreshold(5.centimeters))
        }

    enum class State(val control: ControlRequest) {
        Intaking(VelocityVoltage(6.0)),
        Holding(StaticBrake()),
        Shooting(VelocityVoltage(10.0).withIgnoreHardwareLimits(true)),
        Amping(VelocityVoltage(4.0)),
    }

    var state = State.Intaking

    init {
        val feederConfig =
            TalonFXConfiguration().apply {
                Feedback.SensorToMechanismRatio = 1.0
                Slot0.kP = 10.0
                withHardwareLimitSwitch(
                    HardwareLimitSwitchConfigs().withForwardLimitRemoteCANrange(noteDetector)
                )
            }
        feederMotor.configurator.apply(feederConfig)
    }

    fun stateActions() {
        feederMotor.setControl(state.control)
    }

    val hasNote
        get() = noteDetector.isDetected.value

    private val noteTimer = Timer().also { restart() }

    fun stateTransitions() {
        when (state) {
            State.Intaking -> {
                if (noteDetector.isDetected.value) {
                    state = State.Holding
                }
            }
            State.Holding -> {
                if (!noteDetector.isDetected.value) {
                    state = State.Intaking
                } else if (OI.shoot() && Shooter.atShootingSpeed) {
                    state = State.Shooting
                } else if (OI.shoot() && Superstructure.state == Superstructure.State.AMP) {
                    state = State.Amping
                }
            }
            State.Shooting -> {
                if (noteDetector.isDetected.value) {
                    noteTimer.restart()
                } else if (noteTimer.hasElapsed(0.25)) {
                    state = State.Intaking
                }
            }
            State.Amping -> {
                if (noteDetector.isDetected.value) {
                    noteTimer.restart()
                } else if (noteTimer.hasElapsed(0.75)) {
                    state = State.Intaking
                }
            }
        }
    }
}
