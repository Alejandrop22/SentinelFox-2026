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
import frc.robot.subsystems.Alignment;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.OIConstants;

public class RobotContainer {
    private static final double kDriveScale = 1.0;

    private final Camara m_camara = new Camara();
    private final DriveSubsystem m_robotDrive = new DriveSubsystem(m_camara);
    private final Alignment m_alignment = new Alignment(m_robotDrive, m_camara);
    private final Angular m_angular = new Angular();
    private final Intake m_intake = new Intake();
    private final Shooter m_shooter = new Shooter();
    private final AsistedShooter m_asistedShooter = new AsistedShooter(m_camara);
    private final AuxMotor m_auxMotor = new AuxMotor();
    // Control 0 = drivetrain (driver)
    private final CommandXboxController m_driverController = new CommandXboxController(0);
    // Control 1 = subsistemas (operator)
    private final CommandXboxController m_operatorController = new CommandXboxController(1);

    // --- Angular / Intake ---
    private boolean m_intake100ToggleActive = false;

    // Angular jog (para calibración / reset 0)
    private boolean m_angularJogActive = false;

    private void stopShooterAll() {
        m_asistedShooter.stop();
        m_shooter.stopManualShooter();
    }

    private void setAssistedShooterPercent(double percent) {
        m_shooter.setAssistedShooterPercent(percent);
    }

    public RobotContainer() {
        configureBindings();

        m_robotDrive.setDefaultCommand(
            new RunCommand(
                () -> {
                    double leftY = -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband) * kDriveScale;
                    double leftX = -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband) * kDriveScale;
                    double rot = -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband) * kDriveScale;
                    double rotWithAutoAim = m_robotDrive.getRotationForDrive(rot);
                    m_robotDrive.drive(
                        leftY,
                        leftX,
                        rotWithAutoAim,
                        true);
                },
                m_robotDrive));
    }

    private void configureBindings() {
        // =========================
        // Control 0 (Driver) - Drivetrain
        // =========================

        // Right Bumper: setX (bloqueo de ruedas)
        m_driverController.rightBumper().whileTrue(
            new RunCommand(() -> m_robotDrive.setX(), m_robotDrive)
        );

        // X: reset heading
        m_driverController.x().onTrue(
            new InstantCommand(() -> m_robotDrive.zeroHeading(), m_robotDrive)
        );

        // POVLeft: toggle AutoAim (solo si Tag1)
        m_driverController.povLeft().onTrue(
            new InstantCommand(() -> {
                if (m_robotDrive.isAutoAimEnabled()) {
                    m_robotDrive.setAutoAimEnabled(false);
                } else if (m_camara.hasAutoAimTag()) {
                    m_robotDrive.setAutoAimEnabled(true);
                }
            }, m_robotDrive)
        );

        // POVRight (while held): "Trench" align (1m away, centered, facing the closest visible trench tag)
        m_driverController.povRight().whileTrue(
            m_alignment.alignToTrenchOneMeter()
        );

        // LT (while held): Shooter (manual/assisted) para Control 0
        m_driverController.leftTrigger().whileTrue(
            new RunCommand(
                () -> {
                    if (m_shooter.isEmergencyEnabled()) {
                        stopShooterAll();
                        return;
                    }

                    // Reglas:
                    // 1) AutoAim OFF  -> shooter fijo -0.56
                    // 2) AutoAim ON pero SIN tag o >4m -> shooter fijo -0.8
                    // 3) AutoAim ON y tag válido (<=4m) -> AssistedShooter (fórmula)
                    if (!m_robotDrive.isAutoAimEnabled()) {
                        stopShooterAll();
                        setAssistedShooterPercent(-0.56);
                        return;
                    }

                    // AutoAim ON
                    if (!m_camara.hasAutoAimTag() || m_camara.getAutoAimDistanceM() > 4.0) {
                        stopShooterAll();
                        setAssistedShooterPercent(-0.8);
                        return;
                    }

                    // AutoAim ON + tag dentro de 4m
                    if (m_asistedShooter.canShootNow()) {
                        m_shooter.stopManualShooter();
                        setAssistedShooterPercent(m_asistedShooter.getDesiredPercent());
                    } else {
                        // Si por alguna razón canShootNow() no deja (distancia 0, etc.), usar el fallback fuerte
                        stopShooterAll();
                        setAssistedShooterPercent(-0.8);
                    }
                },
                m_asistedShooter, m_shooter)
        ).onFalse(
            new InstantCommand(() -> {
                stopShooterAll();
            }, m_asistedShooter, m_shooter)
        );

        // B (while held): Banda + AuxMotor para Control 0
        m_driverController.b().whileTrue(
            new RunCommand(() -> {
                m_shooter.startBelt();
                m_auxMotor.startReverseAux();
            }, m_shooter, m_auxMotor)
        ).onFalse(
            new InstantCommand(() -> {
                m_shooter.stopBelt();
                m_auxMotor.stop();
            }, m_shooter, m_auxMotor)
        );

        // =========================
        // Control 1 (Operator) - Subsistemas
        // =========================

        // --- Angular / Intake (CAN 54 + CAN 55) ---

        // Intake: LT (while held) usando intakeReverse() (la velocidad la ajustas en Intake)
        // Si POVUp (100%) está activo, LT no debe pelear: no modifica el motor.
        m_operatorController.leftTrigger().whileTrue(
            new RunCommand(() -> {
                if (m_intake100ToggleActive) {
                    return;
                }
                m_intake.intakeReverse();
            }, m_intake)
        ).onFalse(
            new InstantCommand(() -> {
                if (m_intake100ToggleActive) {
                    // Si sigue activo 100%, dejarlo prendido
                    return;
                }
                m_intake.stop();
            }, m_intake)
        );

        // Intake: POVUp (toggle) 100% forward
        // Al apagar, se apaga el intake (LT ahora es while-held).
        m_operatorController.povUp().onTrue(
            new InstantCommand(() -> {
                m_intake100ToggleActive = !m_intake100ToggleActive;
                if (m_intake100ToggleActive) {
                    // Prender 100% forward
                    m_intake.intakeFullReverse();
                } else {
                    // Apagar 100%
                    m_intake.stop();
                }
            }, m_intake)
        );

        // Intake: LB (while held) reverse
        m_operatorController.leftBumper().whileTrue(
            new RunCommand(() -> m_intake.intakeForward(), m_intake)
        ).onFalse(
            new InstantCommand(() -> m_intake.stop(), m_intake)
        );

        // Angular: A -> posición 0° (home)
        m_operatorController.y().onTrue(
            new InstantCommand(() -> m_angular.irAPosicion((-360*5)), m_angular)
        );

        // Angular: B -> auto-bajar hasta detectar stall y luego subir 2 rotaciones
        m_operatorController.b().onTrue(
            new InstantCommand(() -> m_angular.startAutoDownToStallAndBackoff(), m_angular)
        );

        // Angular: RB (one-shot) -> un ciclo anti-atoradas (solo si ya está abajo)
        m_operatorController.rightBumper().onTrue(
            new InstantCommand(() -> m_angular.setUnjamCycleEnabled(true), m_angular)
        );

        // Angular: Y -> abajo
        m_operatorController.a().onTrue(
            new InstantCommand(() -> m_angular.irAPosicion(-(360.0 * 31)), m_angular)
        );

        // Angular: START -> posición intermedia
        m_operatorController.start().onTrue(
            new InstantCommand(() -> m_angular.irAPosicion(-(360.0 * 23)), m_angular)
        );

        m_operatorController.rightTrigger().whileTrue(
            new RunCommand(
                () -> {
                    if (m_shooter.isEmergencyEnabled()) {
                        stopShooterAll();
                        return;
                    }

                    // Reglas:
                    // 1) AutoAim OFF  -> shooter fijo -0.56
                    // 2) AutoAim ON pero SIN tag o >4m -> shooter fijo -0.8
                    // 3) AutoAim ON y tag válido (<=4m) -> AssistedShooter (fórmula)
                    if (!m_robotDrive.isAutoAimEnabled()) {
                        stopShooterAll();
                        setAssistedShooterPercent(-0.56);
                        return;
                    }

                    // AutoAim ON
                    if (!m_camara.hasAutoAimTag() || m_camara.getAutoAimDistanceM() > 4.0) {
                        stopShooterAll();
                        setAssistedShooterPercent(-0.8);
                        return;
                    }

                    // AutoAim ON + tag dentro de 4m
                    if (m_asistedShooter.canShootNow()) {
                        m_shooter.stopManualShooter();
                        setAssistedShooterPercent(m_asistedShooter.getDesiredPercent());
                    } else {
                        // Si por alguna razón canShootNow() no deja (distancia 0, etc.), usar el fallback fuerte
                        stopShooterAll();
                        setAssistedShooterPercent(-0.8);
                    }
                },
                m_asistedShooter, m_shooter)
        ).onFalse(
            new InstantCommand(() -> {
                stopShooterAll();
            }, m_asistedShooter, m_shooter)
        );
        InstantCommand toggleEmergencyShooterCmd = new InstantCommand(() -> {
            m_asistedShooter.stop();
            m_shooter.toggleEmergencyShooter();
        }, m_shooter, m_asistedShooter);

        m_operatorController.leftStick().onTrue(toggleEmergencyShooterCmd);
        m_operatorController.rightStick().onTrue(toggleEmergencyShooterCmd);

        // X (while held): Banda + AuxMotor
        m_operatorController.x().whileTrue(
            new RunCommand(() -> {
                m_shooter.startBelt();
                m_auxMotor.startReverseAux();
            }, m_shooter, m_auxMotor)
        ).onFalse(
            new InstantCommand(() -> {
                m_shooter.stopBelt();
                m_auxMotor.stop();
            }, m_shooter, m_auxMotor)
        );

        // POVRight (while held): Angular open-loop 20% (SIN PID) para calibración
        // Mientras se sostiene, mueve el Angular; al soltar, lo apaga (se queda donde está).
        m_operatorController.povRight().whileTrue(
            new RunCommand(() -> {
                if (!m_angularJogActive) {
                    m_angularJogActive = true;
                }
                m_angular.setOpenLoopPercent(0.30);
            }, m_angular)
        ).onFalse(
            new InstantCommand(() -> {
                m_angularJogActive = false;
                m_angular.stopOpenLoop();
            }, m_angular)
        );

        // Back button: tomar esta posición actual como 0 del Angular (CAN 54)
        // Nota: 'back()' existe en CommandXboxController para Xbox (View/Back).
        m_operatorController.back().onTrue(
            new InstantCommand(() -> m_angular.resetEncoder(), m_angular)
        );
    }

    public Command getAutonomousCommand() {
        return null;
    }

}