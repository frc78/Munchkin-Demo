package frc.robot.subsystems

import com.ctre.phoenix6.swerve.SwerveRequest
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain
import frc.robot.hid.MunchkinController

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements Subsystem so it can easily
 * be used in command-based projects.
 */
object Drivetrain : TunerSwerveDrivetrain() {

    private val FieldCentric = SwerveRequest.FieldCentric()

    fun stateMachine() {
        // Currently no additional state machine logic beyond what is in the base class
        setControl(
            FieldCentric.withVelocityX(MunchkinController.translationX)
                .withVelocityY(MunchkinController.translationY)
                .withRotationalRate(MunchkinController.rotation)
        )
        if (MunchkinController.resetRotation()) {
            seedFieldCentric()
        }
    }
}
