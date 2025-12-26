package frc.robot.subsystems

import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.Follower
import com.ctre.phoenix6.controls.MotionMagicVoltage
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.InvertedValue
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.wpilibj.RobotController
import edu.wpi.first.wpilibj.simulation.ElevatorSim
import frc.robot.hid.MunchkinButtonBoard
import frc.robot.hid.MunchkinController
import frc.robot.lib.inches
import frc.robot.lib.inchesPerSecond
import frc.robot.lib.kilograms
import frc.robot.lib.meters
import frc.robot.lib.metersPerSecond
import frc.robot.lib.pounds
import frc.robot.lib.rotations
import frc.robot.lib.seconds
import kotlin.math.PI
import org.littletonrobotics.junction.Logger

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
                    ForwardSoftLimitThreshold = heightToSpoolRotations(25.0)
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
                    MotionMagicAcceleration = 1000.0
                    MotionMagicCruiseVelocity = 3.0
                }
                MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive
            }
        leader.configurator.apply(leaderConfig)
        follower.setControl(Follower(11, true))
    }

    fun stateMachine() {
        leader.setControl(PositionControl)
        if (MunchkinController.raiseElevator() || MunchkinButtonBoard.raiseElevator()) {
            PositionControl.withPosition(heightToSpoolRotations(16.0))
        } else if (
            MunchkinController.home() ||
                MunchkinButtonBoard.lowerElevator() ||
                MunchkinButtonBoard.home()
        ) {
            PositionControl.withPosition(0.0)
        }
    }

    val sim by lazy {
        ElevatorSim(
            DCMotor.getFalcon500Foc(2),
            25.0,
            15.pounds.kilograms,
            (1.29 / 2).inches.meters,
            0.0,
            16.inches.meters,
            true,
            0.0,
        )
    }

    fun heightToSpoolRotations(linear: Double): Double {
        return linear / (1.29 * PI)
    }

    val heightMeters
        get() = (leader.position.value.rotations * 1.29 * PI).inches.meters

    private var lastTime = RobotController.getMeasureFPGATime()

    fun simulationPeriodic() {
        Logger.recordOutput("elevator/position", sim.positionMeters.meters)
        leader.simState.setSupplyVoltage(RobotController.getBatteryVoltage())
        sim.setInputVoltage(leader.simState.motorVoltage)
        sim.update((RobotController.getMeasureFPGATime() - lastTime).seconds)
        lastTime = RobotController.getMeasureFPGATime()
        leader.simState.setRawRotorPosition(
            heightToSpoolRotations(sim.positionMeters.meters.inches) * 25.0
        )
        leader.simState.setRotorVelocity(
            heightToSpoolRotations(sim.velocityMetersPerSecond.metersPerSecond.inchesPerSecond) *
                25.0
        )
    }
}
