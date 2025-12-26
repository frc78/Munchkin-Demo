package frc.robot.subsystems

import com.ctre.phoenix6.Utils
import com.ctre.phoenix6.swerve.SwerveRequest
import edu.wpi.first.wpilibj.Notifier
import edu.wpi.first.wpilibj.RobotController
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain
import frc.robot.hid.MunchkinController

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements Subsystem so it can easily
 * be used in command-based projects.
 */
object Drivetrain : TunerSwerveDrivetrain() {

    const val kSimLoopPeriod: Double = 0.004
    private lateinit var m_simNotifier: Notifier
    private var m_lastSimTime = 0.0

    public fun simulationInit() {
        m_lastSimTime = Utils.getCurrentTimeSeconds()

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier =
            Notifier(
                Runnable {
                    val currentTime: Double = Utils.getCurrentTimeSeconds()
                    val deltaTime = currentTime - m_lastSimTime
                    m_lastSimTime = currentTime

                    /* Use the measured time delta, get battery voltage from WPILib */
                    updateSimState(deltaTime, RobotController.getBatteryVoltage())
                }
            )
        m_simNotifier.startPeriodic(kSimLoopPeriod)
    }

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
