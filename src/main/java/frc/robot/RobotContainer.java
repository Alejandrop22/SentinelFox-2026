//Noveno commit

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
    private boolean m_intake60ToggleActive = false;
    private boolean m_intake100ToggleActive = false;
    private boolean m_beltAuxComboToggleActive = false;

    // Angular jog (para calibración / reset 0)
    private boolean m_angularJogActive = false;

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
        // --- Angular / Intake (CAN 54 + CAN 55) ---

        // Intake: LT (toggle) ~60% forward
        // Si POVUp (100%) está activo, LT no lo apaga; solo define el "estado anterior".
        m_driverController.leftTrigger().onTrue(
            new InstantCommand(() -> {
                m_intake60ToggleActive = !m_intake60ToggleActive;

                // Si estamos en 100%, no tocar el motor; solo guardar el estado.
                if (m_intake100ToggleActive) {
                    return;
                }

                if (m_intake60ToggleActive) {
                    m_intake.intakeForward();
                } else {
                    m_intake.stop();
                }
            }, m_intake)
        );

        // Intake: POVUp (toggle) 100% forward
        // Al apagar, regresa al estado anterior: si LT estaba activo, vuelve a ~60%; si no, se apaga.
        m_driverController.povUp().onTrue(
            new InstantCommand(() -> {
                m_intake100ToggleActive = !m_intake100ToggleActive;
                if (m_intake100ToggleActive) {
                    // Prender 100% forward
                    m_intake.intakeFullReverse();
                } else {
                    // Apagar 100%: regresar a estado anterior (LT 60% o apagado)
                    if (m_intake60ToggleActive) {
                        m_intake.intakeForward();
                    } else {
                        m_intake.stop();
                    }
                }
            }, m_intake)
        );

        // Intake: LB (while held) reverse
        m_driverController.leftBumper().whileTrue(
            new RunCommand(() -> m_intake.intakeReverse(), m_intake)
        ).onFalse(
            new InstantCommand(() -> m_intake.stop(), m_intake)
        );

        // Angular: A -> posición 0° (home)
        m_driverController.a().onTrue(
            new InstantCommand(() -> m_angular.irAPosicion((-360*5)), m_angular)
        );

        // Angular: B -> toggle ciclo anti-atoradas (solo si ya está abajo)
        m_driverController.b().onTrue(
            new InstantCommand(() -> m_angular.toggleUnjamCycle(), m_angular)
        );

        // Angular: Y -> abajo
        m_driverController.y().onTrue(
            new InstantCommand(() -> m_angular.irAPosicion(-(360.0 * 28)), m_angular)
        );

        // Right Bumper: setX (bloqueo de ruedas)
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

        // POVDown: toggle combinado Banda + AuxMotor
        m_driverController.povDown().onTrue(
            new InstantCommand(() -> {
                m_beltAuxComboToggleActive = !m_beltAuxComboToggleActive;

                if (m_beltAuxComboToggleActive) {
                    m_shooter.startBelt();
                    m_auxMotor.startReverse50();
                } else {
                    m_shooter.stopBelt();
                    m_auxMotor.stop();
                }
            }, m_shooter, m_auxMotor)
        );

        // POVRight: ya no se usa (se combinó en POVDown)

        // POVRight (while held): Angular open-loop 20% (SIN PID) para calibración
        // Mientras se sostiene, mueve el Angular; al soltar, lo apaga (se queda donde está).
        m_driverController.povRight().whileTrue(
            new RunCommand(() -> {
                if (!m_angularJogActive) {
                    m_angularJogActive = true;
                }
                m_angular.setOpenLoopPercent(0.20);
            }, m_angular)
        ).onFalse(
            new InstantCommand(() -> {
                m_angularJogActive = false;
                m_angular.stopOpenLoop();
            }, m_angular)
        );

        // Back button: tomar esta posición actual como 0 del Angular (CAN 54)
        // Nota: 'back()' existe en CommandXboxController para Xbox (View/Back).
        m_driverController.back().onTrue(
            new InstantCommand(() -> m_angular.resetEncoder(), m_angular)
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