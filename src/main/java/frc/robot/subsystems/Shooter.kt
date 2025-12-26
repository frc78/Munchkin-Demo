package frc.robot.subsystems

import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.ControlRequest
import com.ctre.phoenix6.controls.DutyCycleOut
import com.ctre.phoenix6.controls.Follower
import com.ctre.phoenix6.controls.NeutralOut
import com.ctre.phoenix6.hardware.TalonFX
import edu.wpi.first.units.measure.AngularVelocity
import frc.robot.hid.MunchkinButtonBoard
import frc.robot.hid.MunchkinController

object Shooter {
    private val leader = TalonFX(14)
    private val follower = TalonFX(15)

    init {
        val config = TalonFXConfiguration().apply { Feedback.SensorToMechanismRatio = 2.0 }
        leader.configurator.apply(config)

        follower.setControl(Follower(14, false))
    }

    enum class ShooterState(val control: ControlRequest) {
        Idle(NeutralOut()),
        ShootFast(DutyCycleOut(1.0)),
        ShootMedium(DutyCycleOut(0.66)),
        ShootSlow(DutyCycleOut(0.33)),
    }

    var currentState = ShooterState.Idle
        private set

    val speed: AngularVelocity
        get() = leader.velocity.value

    fun stateMachine() {
        leader.setControl(currentState.control)
        if (MunchkinController.slowShot() || MunchkinButtonBoard.slowShot()) {
            currentState = ShooterState.ShootSlow
        } else if (MunchkinController.mediumShot() || MunchkinButtonBoard.mediumShot()) {
            currentState = ShooterState.ShootMedium
        } else if (MunchkinController.fastShot() || MunchkinButtonBoard.fastShot()) {
            currentState = ShooterState.ShootFast
        } else if (MunchkinController.home() || MunchkinButtonBoard.home()) {
            currentState = ShooterState.Idle
        }
    }
}
