package frc.robot.subsystems

import com.ctre.phoenix6.controls.Follower
import com.ctre.phoenix6.controls.VelocityVoltage
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

    private val shooterFast = VelocityVoltage(6000.0)
    private val shooterMedium = VelocityVoltage(4000.0)
    private val shooterSlow = VelocityVoltage(2000.0)
}
