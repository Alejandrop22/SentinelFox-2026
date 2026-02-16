package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Camara;
import frc.robot.subsystems.Angular;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.AuxMotor;
import frc.robot.subsystems.AsistedShooter;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.OIConstants;

public class RobotContainer {
    private final Camara m_camara = new Camara();
    private final DriveSubsystem m_robotDrive = new DriveSubsystem(m_camara);
    private final Angular m_angular = new Angular();
    private final Intake m_intake = new Intake();
    private final Shooter m_shooter = new Shooter();
    private final AsistedShooter m_asistedShooter = new AsistedShooter(m_camara);
    private final AuxMotor m_auxMotor = new AuxMotor();
    private final CommandXboxController m_driverController =
            new CommandXboxController(Constants.OperatorConstants.kDriverControllerPort);

    // --- Angular / Intake ---
    private boolean m_beltToggleActive = false;
    // Aux motor toggle (CAN 52)
    private boolean m_auxToggleActive = false;

    public RobotContainer() {
        configureBindings();

        // Default drive command for swerve
        m_robotDrive.setDefaultCommand(
            new RunCommand(
                () -> {
                    // Compute the translation magnitude from the left stick (0..1) and report it
                    // to the DriveSubsystem so aim can reduce rotation when translation is high.
                    double leftY = -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband);
                    double leftX = -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband);
                    double transMag = Math.min(1.0, Math.hypot(leftY, leftX));
                    m_robotDrive.setLastTranslationMagnitude(transMag);
                    m_robotDrive.drive(
                        leftY,
                        leftX,
                        // Use the drive subsystem helper so a rotation override (from AimToAprilTag)
                        // can block operator rotation while still allowing translation.
                        m_robotDrive.getRotationForDrive(-MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband)),
                        true);
                },
                m_robotDrive));
    }

    private void configureBindings() {
        // --- Angular / Intake (CAN 54 + CAN 55) ---

        // Intake: LT (while held) forward
        m_driverController.leftTrigger().whileTrue(
            new RunCommand(() -> m_intake.intakeForward(), m_intake)
        ).onFalse(
            new InstantCommand(() -> m_intake.stop(), m_intake)
        );

        // Intake: LB (while held) reverse
        m_driverController.leftBumper().whileTrue(
            new RunCommand(() -> m_intake.intakeReverse(), m_intake)
        ).onFalse(
            new InstantCommand(() -> m_intake.stop(), m_intake)
        );

        // Angular: A -> posición 0° (home)
        m_driverController.a().onTrue(
            new InstantCommand(() -> m_angular.irAPosicion(0.0), m_angular)
        );

        // Angular: Y -> abajo
        m_driverController.y().onTrue(
            new InstantCommand(() -> m_angular.irAPosicion(-(360.0 * 28)), m_angular)
        );

        // Right Bumper: setX (bloqueo de ruedas)
        m_driverController.rightBumper().whileTrue(
            new RunCommand(() -> m_robotDrive.setX(), m_robotDrive)
        );

        // Back button: zero heading (gyro)
        m_driverController.x().onTrue(
            new InstantCommand(() -> m_robotDrive.zeroHeading(), m_robotDrive)
        );

        // Right Trigger:
        // If Tag1 is visible AND within 4m => assisted shooter (distance-based percent).
        // Otherwise => manual shooter (always -0.8).
        m_driverController.rightTrigger().whileTrue(
            new RunCommand(
                () -> {
                    // Emergency has top priority: leave CAN 51 alone.
                    if (m_shooter.isEmergencyEnabled()) {
                        m_asistedShooter.stop();
                        return;
                    }
                    if (m_asistedShooter.canShootNow()) {
                        // Ensure manual isn't running
                        m_shooter.stopManualShooter();

                        // AssistedShooter no controla el motor directamente para evitar doble instancia CAN 51.
                        m_asistedShooter.runAssisted(); // solo telemetry
                        m_shooter.setAssistedShooterPercent(m_asistedShooter.getDesiredPercent());
                    } else {
                        // Ensure assisted isn't running
                        m_asistedShooter.stop();
                        m_shooter.startManualShooter();
                    }
                },
                m_asistedShooter, m_shooter)
        ).onFalse(
            new InstantCommand(() -> {
                m_asistedShooter.stop();
                m_shooter.stopManualShooter();
            }, m_asistedShooter, m_shooter)
        );


        // Stick click (L3 / R3): emergency shooter toggle (-0.5)
        // If this is enabled, we stop assisted shooter so only one thing drives CAN 51.
        InstantCommand toggleEmergencyShooterCmd = new InstantCommand(() -> {
            // Turning on/off is handled in Shooter.
            m_asistedShooter.stop();
            m_shooter.toggleEmergencyShooter();
        }, m_shooter, m_asistedShooter);

        m_driverController.leftStick().onTrue(toggleEmergencyShooterCmd);
        m_driverController.rightStick().onTrue(toggleEmergencyShooterCmd);

        // Cruzeta abajo (POV 180): banda a 50% en toggle
        m_driverController.povDown().onTrue(
            new InstantCommand(() -> {
                if (m_beltToggleActive) {
                    m_shooter.stopBelt();
                } else {
                    m_shooter.startBelt();
                }
                m_beltToggleActive = !m_beltToggleActive;
            }, m_shooter)
        );

        // Cruzeta derecha (POV 90): toggle AuxMotor (CAN 52) reverse at 50%
        m_driverController.povRight().onTrue(
            new InstantCommand(() -> {
                if (m_auxToggleActive) {
                    m_auxMotor.stop();
                } else {
                    m_auxMotor.startReverse50();
                }
                m_auxToggleActive = !m_auxToggleActive;
            }, m_auxMotor)
        );

        // Cruzeta izquierda (POV 270): toggle AutoAim hacia AprilTag 1.
        // Solo activa si en ese momento se detecta el tag 1.
        m_driverController.povLeft().onTrue(
            new InstantCommand(() -> {
                if (m_robotDrive.isAutoAimEnabled()) {
                    m_robotDrive.setAutoAimEnabled(false);
                } else if (m_camara.hasTag1()) {
                    m_robotDrive.setAutoAimEnabled(true);
                }
            }, m_robotDrive)
        );
    }

    public Command getAutonomousCommand() {
        return null;
    }

}