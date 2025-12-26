package frc.robot.subsystems

import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.DutyCycleOut
import com.ctre.phoenix6.controls.MotionMagicVoltage
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.GravityTypeValue
import com.ctre.phoenix6.signals.InvertedValue
import edu.wpi.first.units.Units.Degrees
import frc.robot.hid.MunchkinButtonBoard
import frc.robot.hid.MunchkinController
import frc.robot.lib.degrees

object Wrist {
    val MAX_ANGLE = Degrees.of(55.0)

    val motor = TalonFX(13) // TODO Create a new TalonFX with id 13
    private val positionControl = MotionMagicVoltage(55.degrees)

    init {
        val config = TalonFXConfiguration()

        config.Feedback.SensorToMechanismRatio = 3 * 3 * 4 * 36 / 12.0
        config.SoftwareLimitSwitch.withReverseSoftLimitEnable(true)
            .withForwardSoftLimitEnable(true)
            .withReverseSoftLimitThreshold(0.degrees)
            .withForwardSoftLimitThreshold(55.degrees)
        config.Slot0.kP = 500.0
        config.Slot0.kV = 12.19
        config.Slot0.kA = 0.02
        config.Slot0.kG = 0.16
        config.Slot0.GravityType = GravityTypeValue.Arm_Cosine
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive
        config.ClosedLoopGeneral.ContinuousWrap = true

        motor.setPosition(MAX_ANGLE)
        motor.configurator.apply(config)
    }

    enum class WristState {
        Idle,
        Raising,
        Lowering,
    }

    private var currentState = WristState.Idle

    private val RaiseControl = DutyCycleOut(0.4)
    private val LowerControl = DutyCycleOut(-0.4)

    fun stateMachine() {
        when (currentState) {
            WristState.Idle -> {
                motor.setControl(positionControl)
                if (MunchkinController.home()) {
                    positionControl.withPosition(MAX_ANGLE)
                }
                if (MunchkinController.lowerWrist() || MunchkinButtonBoard.lowerWrist()) {
                    currentState = WristState.Lowering
                } else if (MunchkinController.raiseWrist() || MunchkinButtonBoard.raiseWrist()) {
                    currentState = WristState.Raising
                }
            }
            WristState.Lowering -> {
                motor.setControl(LowerControl)
                if (!MunchkinController.lowerWrist() && !MunchkinButtonBoard.lowerWrist()) {
                    positionControl.withPosition(motor.position.value)
                    currentState = WristState.Idle
                }
            }
            WristState.Raising -> {
                motor.setControl(RaiseControl)
                if (!MunchkinController.raiseWrist() && !MunchkinButtonBoard.raiseWrist()) {
                    positionControl.withPosition(motor.position.value)
                    currentState = WristState.Idle
                }
            }
        }
    }
}
