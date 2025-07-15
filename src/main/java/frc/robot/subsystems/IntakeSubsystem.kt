package frc.robot.subsystems

import com.ctre.phoenix6.controls.Follower
import com.ctre.phoenix6.hardware.TalonFX
import edu.wpi.first.wpilibj.motorcontrol.PWMTalonFX
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.StartEndCommand
import edu.wpi.first.wpilibj2.command.SubsystemBase;

object IntakeSubsystem : SubsystemBase() {
val leader = TalonFX(9)
    val follower = TalonFX(10)
    init {
        follower.setControl(Follower(9, false))
    }
fun Intake(): Command{
    return startEnd({leader.set(0.4)},{leader.set(0.0)})
}
}