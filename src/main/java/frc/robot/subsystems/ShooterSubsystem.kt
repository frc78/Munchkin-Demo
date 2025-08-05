package frc.robot.subsystems

import com.ctre.phoenix6.controls.DutyCycleOut
import com.ctre.phoenix6.controls.Follower
import com.ctre.phoenix6.hardware.TalonFX
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands.startEnd

object ShooterSubsystem {
    private val leader = TalonFX(14)
    private val follower = TalonFX(15)

    init {
        follower.setControl(Follower(14, false))
    }

    fun shootFast(): Command {
        return startEnd({ leader.setControl(shooterFast) }, { leader.set(0.0) })
    }

    fun shootMedium(): Command {
        return startEnd({ leader.setControl(shooterMedium) }, { leader.set(0.0) })
    }

    fun shootSlow(): Command {
        return startEnd({ leader.setControl(shooterSlow) }, { leader.set(0.0) })
    }

    private val shooterFast = DutyCycleOut(1.0)
    private val shooterMedium = DutyCycleOut(2.0 / 3.0)
    private val shooterSlow = DutyCycleOut(1.0 / 3.0)
}
