package frc.robot.subsystems

import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.Follower
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage
import com.ctre.phoenix6.controls.MotionMagicVoltage
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.GravityTypeValue
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.MotorAlignmentValue
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.system.plant.LinearSystemId
import edu.wpi.first.units.Units.Degrees
import edu.wpi.first.wpilibj.RobotController
import edu.wpi.first.wpilibj.simulation.ElevatorSim
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim
import frc.robot.hid.OI
import frc.robot.lib.degrees
import frc.robot.lib.inches
import frc.robot.lib.kilograms
import frc.robot.lib.meters
import frc.robot.lib.metersPerSecond
import frc.robot.lib.pounds
import frc.robot.lib.radians
import frc.robot.lib.radiansPerSecond
import frc.robot.lib.rotations
import frc.robot.lib.seconds
import frc.robot.lib.toAngle
import frc.robot.lib.toAngularVelocity
import frc.robot.subsystems.Superstructure.State.AMP
import frc.robot.subsystems.Superstructure.State.CLIMB
import frc.robot.subsystems.Superstructure.State.INTAKE
import frc.robot.subsystems.Superstructure.State.SHOOT
import kotlin.math.PI
import org.littletonrobotics.junction.Logger

object Superstructure {

    enum class State {
        INTAKE,
        SHOOT,
        AMP,
        CLIMB,
    }

    private val elevatorLeader = TalonFX(11)
    private val elevatorFollower = TalonFX(12)
    private val elevatorControl = MotionMagicExpoVoltage    (0.0)
    private val DRUM_RADIUS = (1.29 / 2).inches
    private const val ELEVATOR_GEAR_RATIO = 5.0

    var state = INTAKE

    init {
        val elevatorConfig =
            TalonFXConfiguration().apply {
                Feedback.SensorToMechanismRatio = ELEVATOR_GEAR_RATIO
                SoftwareLimitSwitch.apply {
                    ReverseSoftLimitThreshold = 0.0
                    ForwardSoftLimitThreshold = 16.9.inches.toAngle(DRUM_RADIUS).rotations
                    ForwardSoftLimitEnable = true
                    ReverseSoftLimitEnable = true
                }
                Slot0.apply {
                    // Scaled from 25:1 SysId gains; re-characterize at 5:1
                    kP = 334.2
                    kI = 0.0
                    kD = 8.71
                    kS = 0.22964
                    kV = 0.1419
                    kA = 0.0940
                    kG = 0.6006
                }
                MotionMagic.apply {
                    MotionMagicExpo_kA = 0.094
                    MotionMagicExpo_kV = 0.1419
                }
                MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive
            }
        elevatorLeader.configurator.apply(elevatorConfig)
        elevatorFollower.setControl(Follower(11, MotorAlignmentValue.Opposed))
    }

    private val MAX_WRIST_ANGLE = Degrees.of(55.0)

    private const val WRIST_GEAR_RATIO = 3.0 * 3.0 * 4.0 * 36.0 / 12.0 // 108

    val wristMotor = TalonFX(13)
    private val wristControl = MotionMagicExpoVoltage(0.0)

    init {
        val wristConfig =
            TalonFXConfiguration().apply {
                Feedback.SensorToMechanismRatio = WRIST_GEAR_RATIO
                SoftwareLimitSwitch.withReverseSoftLimitEnable(true)
                    .withForwardSoftLimitEnable(true)
                    .withReverseSoftLimitThreshold(0.degrees)
                    .withForwardSoftLimitThreshold(MAX_WRIST_ANGLE)
                Slot0.kP = 100.0
                Slot0.kV = 12.19
                Slot0.kA = 0.02
                Slot0.kG = 0.16
                Slot0.GravityType = GravityTypeValue.Arm_Cosine
                MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive
                ClosedLoopGeneral.ContinuousWrap = false
                MotionMagic.MotionMagicExpo_kA = 0.02
                MotionMagic.MotionMagicExpo_kV = 12.19
            }

        wristMotor.configurator.apply(wristConfig)
    }

    fun stateActions() {
        when (state) {
            INTAKE -> {
                elevatorControl.withPosition(0.0)
                wristControl.withPosition(0.degrees)
            }

            SHOOT -> {
                elevatorControl.withPosition(0.0)
            }

            AMP -> {
                elevatorControl.withPosition(16.3.inches.toAngle(DRUM_RADIUS))
                wristControl.withPosition(35.degrees)
            }

            CLIMB -> {
                elevatorControl.withPosition(16.9.inches.toAngle(DRUM_RADIUS))
                wristControl.withPosition(0.degrees)
            }
        }
        elevatorLeader.setControl(elevatorControl)
        wristMotor.setControl(wristControl)
    }

    fun stateTransitions() {
        when (state) {
            INTAKE -> {
                if (Feeder.state == Feeder.State.Holding) {
                    state = SHOOT
                } else if (OI.climb()) {
                    state = CLIMB
                }
            }

            SHOOT -> {
                if (OI.amp()) {
                    state = AMP
                } else if (OI.climb()) {
                    state = CLIMB
                } else if (Feeder.state == Feeder.State.Intaking) {
                    state = INTAKE
                }
            }

            AMP,
            CLIMB -> Unit
        }
        if (OI.home()) {
            state = INTAKE
        }
    }

    val atPosition
        get() = elevatorLeader.closedLoopError.value < 1 && wristMotor.closedLoopError.value < 1

    val sim by lazy {
        ElevatorSim(
            DCMotor.getFalcon500Foc(2),
            ELEVATOR_GEAR_RATIO,
            15.pounds.kilograms,
            (1.29 / 2).inches.meters,
            0.0,
            16.inches.meters,
            true,
            0.0,
        )
    }

    val heightMeters
        get() = (elevatorLeader.position.value.rotations * 1.29 * PI).inches.meters

    val wristAngle
        get() = wristMotor.position.value

    private var lastTime = RobotController.getMeasureFPGATime()

    fun simulationPeriodic() {
        val now = RobotController.getMeasureFPGATime()
        val dt = (now - lastTime).seconds
        lastTime = now

        Logger.recordOutput("elevator/position", sim.positionMeters.meters)
        elevatorLeader.simState.setSupplyVoltage(RobotController.getBatteryVoltage())
        sim.setInputVoltage(elevatorLeader.simState.motorVoltage)
        sim.update(dt)
        elevatorLeader.simState.setRawRotorPosition(
            sim.positionMeters.meters.toAngle(DRUM_RADIUS) * ELEVATOR_GEAR_RATIO
        )
        elevatorLeader.simState.setRotorVelocity(
            sim.velocityMetersPerSecond.metersPerSecond.toAngularVelocity(DRUM_RADIUS) * ELEVATOR_GEAR_RATIO
        )

        Logger.recordOutput("wrist/angle", wristMotor.position.value)
        wristMotor.simState.setSupplyVoltage(RobotController.getBatteryVoltage())
        wristSim.setInputVoltage(wristMotor.simState.motorVoltage)
        wristSim.update(dt)
        wristMotor.simState.setRawRotorPosition((wristSim.angleRads * WRIST_GEAR_RATIO).radians)
        wristMotor.simState.setRotorVelocity(
            (wristSim.velocityRadPerSec * WRIST_GEAR_RATIO).radiansPerSecond
        )
    }

    val wristSim by lazy {
        SingleJointedArmSim(
            // Phoenix gains are per rotation; the sim works in radians
            LinearSystemId.identifyPositionSystem(12.19 / (2 * PI), 0.02 / (2 * PI)),
            DCMotor.getFalcon500Foc(1),
            WRIST_GEAR_RATIO,
            0.25,
            0.0,
            55.degrees.radians,
            true,
            0.0,
        )
    }
}
