package frc.robot.subsystems

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs
import com.ctre.phoenix6.controls.ControlRequest
import com.ctre.phoenix6.controls.DutyCycleOut
import com.ctre.phoenix6.controls.Follower
import com.ctre.phoenix6.controls.NeutralOut
import com.ctre.phoenix6.hardware.TalonFX
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.wpilibj.Timer
import frc.robot.hid.MunchkinButtonBoard
import frc.robot.hid.MunchkinController

object Intake {
    private val leader = TalonFX(9)
    private val follower = TalonFX(10)

    init {
        leader.configurator.apply(ClosedLoopRampsConfigs().withVoltageClosedLoopRampPeriod(1.0))
        follower.setControl(Follower(9, false))
    }

    var currentState = IntakeState.Idle
        private set

    enum class IntakeState(val control: ControlRequest) {
        Idle(NeutralOut()),
        Intake(DutyCycleOut(0.4)),
        Outtake(DutyCycleOut(-0.4)),
    }

    val speed: AngularVelocity
        get() = leader.velocity.value

    private val outtakeTimer = Timer()

    fun stateMachine() {
        leader.setControl(currentState.control)
        when (currentState) {
            IntakeState.Idle -> {
                if (MunchkinController.intake() || MunchkinButtonBoard.intake()) {
                    currentState = IntakeState.Intake
                }
            }
            IntakeState.Intake -> {
                if (MunchkinController.home() || MunchkinButtonBoard.home()) {
                    currentState = IntakeState.Idle
                } else if (Feeder.hasNote) {
                    outtakeTimer.restart()
                    currentState = IntakeState.Outtake
                }
            }
            IntakeState.Outtake -> {
                // no transition to home, always outtake for a bit
                if (outtakeTimer.hasElapsed(2.0)) {
                    currentState = IntakeState.Idle
                }
            }
        }
    }
}
