package frc.robot.subsystems

import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.Follower
import com.ctre.phoenix6.controls.MotionMagicVoltage
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.InvertedValue
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase

object Elevator : SubsystemBase() {

    private val leader = TalonFX(11)
    private val leaderConfig = TalonFXConfiguration()
    private val follower = TalonFX(12)

    init {
        leaderConfig.Feedback.SensorToMechanismRatio = 25.0 / (Math.PI * 1.29)
        leaderConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.0
        leaderConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 16.0
        leaderConfig.Slot0.kP = 66.84
        leaderConfig.Slot0.kI = 0.0
        leaderConfig.Slot0.kD = 1.7421
        leaderConfig.Slot0.kS = 0.22964
        leaderConfig.Slot0.kV = 0.70964
        leaderConfig.Slot0.kA = 0.018805
        leaderConfig.Slot0.kG = 0.12011
        leaderConfig.MotionMagic.MotionMagicAcceleration = 80.0
        leaderConfig.MotionMagic.MotionMagicCruiseVelocity = 15.0
        leaderConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive
        leaderConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true
        leaderConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true
        leader.configurator.apply(leaderConfig)
        follower.setControl(Follower(11, true))
    }

    val positionControl = MotionMagicVoltage(0.0)

    fun elevatorUp(): Command {
        return runOnce { leader.setControl(positionControl.withPosition(16.0)) }
    }

    fun elevatorDown(): Command {
        return runOnce { leader.setControl(positionControl.withPosition(0.0)) }
    }
}
