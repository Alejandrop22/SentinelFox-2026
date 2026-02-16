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

    private boolean m_intakeToggleActive = false;
    private boolean m_intakeFullToggleActive = false;
    private boolean m_beltToggleActive = false;
    private boolean m_auxToggleActive = false;

    public RobotContainer() {
        configureBindings();

        m_robotDrive.setDefaultCommand(
            new RunCommand(
                () -> {
                    double leftY = -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband);
                    double leftX = -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband);
                    double transMag = Math.min(1.0, Math.hypot(leftY, leftX));
                    m_robotDrive.setLastTranslationMagnitude(transMag);
                    m_robotDrive.drive(
                        leftY,
                        leftX,
                        m_robotDrive.getRotationForDrive(-MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband)),
                        true);
                },
                m_robotDrive));
    }

    private void configureBindings() {
        m_driverController.rightBumper().whileTrue(
            new RunCommand(() -> m_robotDrive.setX(), m_robotDrive)
        );
        m_driverController.x().onTrue(
            new InstantCommand(() -> m_robotDrive.zeroHeading(), m_robotDrive)
        );

        m_driverController.rightTrigger().whileTrue(
            new RunCommand(
                () -> {
                    if (m_shooter.isEmergencyEnabled()) {
                        m_asistedShooter.stop();
                        return;
                    }
                    if (m_asistedShooter.canShootNow()) {
                        m_shooter.stopManualShooter();
                        m_shooter.setAssistedShooterPercent(m_asistedShooter.getDesiredPercent());
                    } else {
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
        InstantCommand toggleEmergencyShooterCmd = new InstantCommand(() -> {
            m_asistedShooter.stop();
            m_shooter.toggleEmergencyShooter();
        }, m_shooter, m_asistedShooter);

        m_driverController.leftStick().onTrue(toggleEmergencyShooterCmd);
        m_driverController.rightStick().onTrue(toggleEmergencyShooterCmd);

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