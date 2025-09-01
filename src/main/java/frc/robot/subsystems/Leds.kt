package frc.robot.subsystems

import com.ctre.phoenix6.controls.SolidColor
import com.ctre.phoenix6.controls.StrobeAnimation
import com.ctre.phoenix6.hardware.CANdle
import com.ctre.phoenix6.signals.RGBWColor
import edu.wpi.first.units.Units.Hertz
import edu.wpi.first.units.Units.Seconds
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.util.Color
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.Subsystem

object Leds : Subsystem {
    val candle = CANdle(1)

    val colorControl = SolidColor(0, 7)

    init {
        defaultCommand =
            this.runOnce {
                    candle.setControl(
                        colorControl.withColor(
                            if (
                                DriverStation.getAlliance().orElse(DriverStation.Alliance.Red) ==
                                    DriverStation.Alliance.Red
                            ) {
                                RGBWColor(Color.kRed)
                            } else {
                                RGBWColor(Color.kBlue)
                            }
                        )
                    )
                }
                .andThen(Commands.idle())
                .ignoringDisable(true)
    }

    private val greenFlash =
        StrobeAnimation(0, 7).withColor(RGBWColor(Color.kGreen)).withFrameRate(Hertz.of(10.0))

    fun flashGreen(): Command {
        return this.runOnce { candle.setControl(greenFlash) }
            .andThen(Commands.idle())
            .withTimeout(Seconds.of(1.0))
    }
}
