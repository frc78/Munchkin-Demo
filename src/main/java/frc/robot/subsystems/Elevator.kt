package frc.robot.subsystems

import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.Follower
import com.ctre.phoenix6.controls.MotionMagicVoltage
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.InvertedValue
import frc.robot.hid.MunchkinButtonBoard
import frc.robot.hid.MunchkinController

object Elevator {

  private val PositionControl = MotionMagicVoltage(0.0)

  private val leader = TalonFX(11)
  private val follower = TalonFX(12)

  init {
    val leaderConfig =
        TalonFXConfiguration().apply {
          Feedback.SensorToMechanismRatio = 25.0
          SoftwareLimitSwitch.apply {
            ReverseSoftLimitThreshold = 0.0
            ForwardSoftLimitThreshold = 16.0
            ForwardSoftLimitEnable = true
            ReverseSoftLimitEnable = true
          }
          Slot0.apply {
            kP = 66.84
            kI = 0.0
            kD = 1.7421
            kS = 0.22964
            kV = 0.70964
            kA = 0.018805
            kG = 0.12011
          }
          MotionMagic.apply {
            MotionMagicAcceleration = 40.0
            MotionMagicCruiseVelocity = 15.0
          }
          MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive
        }
    leader.configurator.apply(leaderConfig)
    follower.setControl(Follower(11, true))
  }

  fun stateMachine() {
    leader.setControl(PositionControl)
    if (MunchkinController.raiseElevator() || MunchkinButtonBoard.raiseElevator()) {
      PositionControl.withPosition(16.0)
    } else if (
        MunchkinController.home() ||
            MunchkinButtonBoard.lowerElevator() ||
            MunchkinButtonBoard.home()
    ) {
      PositionControl.withPosition(0.0)
    }
  }
}
