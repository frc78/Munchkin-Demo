package frc.robot.subsystems

import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.DutyCycleOut
import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.GravityTypeValue
import com.ctre.phoenix6.signals.InvertedValue
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.system.plant.LinearSystemId
import edu.wpi.first.units.Units.Degrees
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.RobotController
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim
import frc.robot.hid.MunchkinButtonBoard
import frc.robot.hid.MunchkinController
import frc.robot.lib.degrees
import frc.robot.lib.radians
import frc.robot.lib.radiansPerSecond
import frc.robot.lib.seconds
import org.littletonrobotics.junction.Logger

object Wrist {
    private val MAX_ANGLE = Degrees.of(55.0)

    private const val GEAR_RATIO = 3.0 * 3.0 * 4.0 * 36.0 / 12.0

    val motor = TalonFX(13)
    private val positionControl = PositionVoltage(MAX_ANGLE)

    init {
        val config =
            TalonFXConfiguration().apply {
                Feedback.SensorToMechanismRatio = GEAR_RATIO
                SoftwareLimitSwitch.withReverseSoftLimitEnable(true)
                    .withForwardSoftLimitEnable(true)
                    .withReverseSoftLimitThreshold(0.degrees)
                    .withForwardSoftLimitThreshold(MAX_ANGLE)
                Slot0.kP = 500.0
                Slot0.kV = 12.19
                Slot0.kA = 0.02
                Slot0.kG = 0.16
                Slot0.GravityType = GravityTypeValue.Arm_Cosine
                MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive
                ClosedLoopGeneral.ContinuousWrap = false
            }

        if (RobotBase.isSimulation()) {
            motor.simState.setRawRotorPosition(MAX_ANGLE * GEAR_RATIO)
        }
        motor.configurator.apply(config)
        if (RobotBase.isReal()) {
            motor.setPosition(MAX_ANGLE)
        }
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
                if (
                    MunchkinController.home() ||
                        MunchkinButtonBoard.home() ||
                        MunchkinController.intake() ||
                        MunchkinButtonBoard.intake()
                ) {
                    positionControl.withPosition(MAX_ANGLE)
                }
                // No point in moving wrist if there's no note.
                if (IntakeFeederShooter.hasNote) {
                    if (MunchkinController.lowerWrist() || MunchkinButtonBoard.lowerWrist()) {
                        currentState = WristState.Lowering
                    } else if (
                        MunchkinController.raiseWrist() || MunchkinButtonBoard.raiseWrist()
                    ) {
                        currentState = WristState.Raising
                    }
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

    val angle: Angle
        get() = motor.position.value

    val sim by lazy {
        SingleJointedArmSim(
            LinearSystemId.identifyPositionSystem(12.19, 0.02),
            DCMotor.getFalcon500Foc(1),
            GEAR_RATIO,
            0.25,
            0.0,
            55.degrees.radians,
            false,
            55.degrees.radians,
        )
    }

    private var lastTime = RobotController.getMeasureFPGATime()

    fun simulationPeriodic() {
        Logger.recordOutput("wrist/angle", motor.position.value)
        Logger.recordOutput("wrist/state", currentState)
        Logger.recordOutput("wrist/simAngle", sim.angleRads.radians)
        motor.simState.setSupplyVoltage(RobotController.getBatteryVoltage())
        sim.setInputVoltage(motor.simState.motorVoltage)
        sim.update((RobotController.getMeasureFPGATime() - lastTime).seconds)
        lastTime = RobotController.getMeasureFPGATime()
        motor.simState.setRawRotorPosition((sim.angleRads * GEAR_RATIO).radians)
        motor.simState.setRotorVelocity((sim.velocityRadPerSec * GEAR_RATIO).radiansPerSecond)
    }
}
