package frc.robot.subsystems

import com.ctre.phoenix6.swerve.SwerveRequest
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DriverStation.Alliance
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Subsystem
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain
import java.util.function.Consumer
import java.util.function.Supplier

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements Subsystem so it can easily
 * be used in command-based projects.
 */
object Drivetrain : TunerSwerveDrivetrain(), Subsystem {
    /* Keep track if we've ever applied the operator perspective before or not */
    private var m_hasAppliedOperatorPerspective = false

    /**
     * Returns a command that applies the specified control request to this swerve drivetrain.
     *
     * @param request Function returning the request to apply
     * @return Command to run
     */
    fun applyRequest(requestSupplier: Supplier<SwerveRequest>): Command {
        return run { this.setControl(requestSupplier.get()) }
    }

    override fun periodic() {
        /*
         * Periodically try to apply the operator perspective.
         * If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
         * This allows us to correct the perspective in case the robot code restarts mid-match.
         * Otherwise, only check and apply the operator perspective if the DS is disabled.
         * This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
         */
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance()
                .ifPresent(
                    Consumer { allianceColor: Alliance? ->
                        setOperatorPerspectiveForward(
                            if (allianceColor == Alliance.Red) kRedAlliancePerspectiveRotation
                            else kBlueAlliancePerspectiveRotation
                        )
                        m_hasAppliedOperatorPerspective = true
                    }
                )
        }
    }

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private val kBlueAlliancePerspectiveRotation = Rotation2d.kZero

    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private val kRedAlliancePerspectiveRotation = Rotation2d.k180deg
}
