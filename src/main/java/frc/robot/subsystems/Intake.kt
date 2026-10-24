package frc.robot.subsystems

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs
import com.ctre.phoenix6.controls.Follower
import com.ctre.phoenix6.controls.TorqueCurrentFOC
import com.ctre.phoenix6.controls.VelocityVoltage
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.MotorAlignmentValue
import edu.wpi.first.wpilibj.Timer
import frc.robot.lib.amps
import frc.robot.lib.rpm

object Intake {

    private val intakeLeader = TalonFX(9)
    private val intakeFollower = TalonFX(10)

    init {
        intakeLeader.configurator.apply(
            ClosedLoopRampsConfigs().withVoltageClosedLoopRampPeriod(1.0)
        )
        intakeFollower.setControl(Follower(intakeLeader.deviceID, MotorAlignmentValue.Aligned))
    }

    enum class State {
        /** The intake is running at full speed to intake a note. */
        FULL_INTAKE,
        /**
         * The intake is running in current mode to pull a note into the intake but not into the
         * feeder
         */
        HOLDING,
        /** Ejecting a note from the intake */
        EJECTING,
    }

    var state = State.FULL_INTAKE

    var holdTimer = Timer().also { it.start() }
    var ejectTimer = Timer().also { it.start() }

    val holdingNote
        get() = intakeLeader.velocity.value < 10.rpm && intakeLeader.torqueCurrent.value > 10.amps

    val robotReadyForIntaking
        get() =
            Feeder.state == Feeder.State.Intaking &&
                Superstructure.state == Superstructure.State.INTAKE &&
                Superstructure.atPosition

    fun stateActions() {
        when (state) {
            State.FULL_INTAKE -> {
                intakeLeader.setControl(VelocityVoltage(6.0))
            }
            State.HOLDING -> {
                intakeLeader.setControl(TorqueCurrentFOC(15.amps))
            }
            State.EJECTING -> {
                intakeLeader.setControl(VelocityVoltage(-6.0))
            }
        }
    }

    fun stateTransitions() {
        when (state) {
            State.FULL_INTAKE -> {
                if (!robotReadyForIntaking) {
                    state = State.HOLDING
                }
            }
            State.HOLDING -> {
                // The rules say you can't hold a second note for more than 3 seconds.
                // 2.8 gives us time to eject
                if (!holdingNote || !Feeder.hasNote) {
                    holdTimer.restart()
                } else if (holdTimer.hasElapsed(2.8)) {
                    state = State.EJECTING
                    ejectTimer.restart()
                } else if (robotReadyForIntaking) {
                    state = State.FULL_INTAKE
                }
            }
            State.EJECTING -> {
                if (ejectTimer.hasElapsed(0.25)) {
                    state = State.FULL_INTAKE
                }
            }
        }
    }
}
