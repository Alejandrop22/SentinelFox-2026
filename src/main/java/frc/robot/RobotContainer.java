package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Elevador;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.AuxMotor;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.AutoDriveToAprilTag;
import frc.robot.commands.AimToAprilTag;

public class RobotContainer {
    private final DriveSubsystem m_robotDrive = new DriveSubsystem();
    private final Elevador m_elevador = new Elevador();
    private final Intake m_intake = new Intake();
    private final Shooter m_shooter = new Shooter();
    private final AuxMotor m_auxMotor = new AuxMotor();
    private final CommandXboxController m_driverController =
            new CommandXboxController(Constants.OperatorConstants.kDriverControllerPort);

    private static final double POS_ABAJO_GRADOS = -(360.0 * 27);
    private static final double POS_ARRIBA_CICLO = POS_ABAJO_GRADOS + (360.0 * 10);

    private final double[] m_posOriginal = new double[1];
    private boolean m_bToggleActive = false;
    private boolean m_intakeToggleActive = false;
    private boolean m_intakeFullToggleActive = false;
    private boolean m_beltToggleActive = false;
    private final Command m_bToggleCommand = Commands.sequence(
        new InstantCommand(() -> m_posOriginal[0] = POS_ABAJO_GRADOS),
        new InstantCommand(() -> m_elevador.irAPosicion(POS_ARRIBA_CICLO)),
        Commands.waitUntil(() -> m_elevador.atSetpoint()),
        Commands.waitSeconds(0.1),
        new InstantCommand(() -> m_elevador.irAPosicion(POS_ABAJO_GRADOS)),
        Commands.waitUntil(() -> m_elevador.atSetpoint()),
        Commands.waitSeconds(0.1)
    ).repeatedly();
    // Aim toggle state: we hold a reference to the command and check isScheduled()
    private Command m_aimCommand = null;
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
        // --- TUS NUEVOS CONTROLES ---

        // Botón Y: IR ARRIBA (Regresar a 0)
        m_driverController.y().onTrue(
            new InstantCommand(() -> {
                if (m_bToggleActive) {
                    m_bToggleCommand.cancel();
                    m_bToggleActive = false;
                }
                m_elevador.irAPosicion(-(360*5));
            })
        );

        // Botón A: IR ABAJO (Bajar 23 vueltas = -8280 grados)
        // Usamos negativo porque 0 es arriba
        m_driverController.a().onTrue(
            new InstantCommand(() -> {
                if (m_bToggleActive) {
                    m_bToggleCommand.cancel();
                    m_bToggleActive = false;
                }
                m_elevador.irAPosicion(-(360*28));
            })
        );

        // Botón Start (Tres rayitas): RESET DE EMERGENCIA
        // Si sientes que el 0 ya no es 0, pícale aquí
        m_driverController.start().onTrue(
            new InstantCommand(() -> m_elevador.resetEncoder())
        );

        // Right Bumper: setX (bloqueo de ruedas)
        m_driverController.rightBumper().whileTrue(
            new RunCommand(() -> m_robotDrive.setX(), m_robotDrive)
        );

        // Back button: zero heading (gyro)
        m_driverController.x().onTrue(
            new InstantCommand(() -> m_robotDrive.zeroHeading(), m_robotDrive)
        );

        // Botón B: toggle infinito (de -9540 a -7740 y regreso) hasta volver a presionar
        m_driverController.b().onTrue(
            new InstantCommand(() -> {
                if (m_bToggleActive) {
                    m_bToggleCommand.cancel();
                    m_elevador.irAPosicion(POS_ABAJO_GRADOS);
                } else if (m_elevador.isAbajo()) {
                    m_bToggleCommand.schedule();
                }
                m_bToggleActive = !m_bToggleActive;
            })
        );

        // Left Trigger: intake hacia atras 60% en toggle
        m_driverController.leftTrigger().onTrue(
            new InstantCommand(() -> {
                if (m_intakeToggleActive) {
                    m_intake.stop();
                } else {
                    m_intake.intakeReverse();
                }
                m_intakeToggleActive = !m_intakeToggleActive;
            }, m_intake)
        );

        // Right Trigger: lanzador (motor CAN 51)
        m_driverController.rightTrigger().onTrue(
            new InstantCommand(() -> m_shooter.startLauncher(), m_shooter)
        ).onFalse(
            new InstantCommand(() -> m_shooter.stopLauncher(), m_shooter)
        );

        // LB: intake hacia adelante 40% mientras se presiona
        m_driverController.leftBumper().whileTrue(
            new InstantCommand(() -> m_intake.intakeForward(), m_intake)
        ).onFalse(
            new InstantCommand(() -> m_intake.stop(), m_intake)
        );

        // Cruzeta arriba (POV 0): intake a -100% en toggle
        m_driverController.povUp().onTrue(
            new InstantCommand(() -> {
                if (m_intakeFullToggleActive) {
                    if (m_intakeToggleActive) {
                        m_intake.intakeReverse(); // regresar a -60%
                    } else {
                        m_intake.stop();
                    }
                } else {
                    m_intake.intakeFullReverse(); // -100%
                }
                m_intakeFullToggleActive = !m_intakeFullToggleActive;
            }, m_intake)
        );

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

        // Cruzeta izquierda (POV 270): Aim toggle to AprilTag (press to start/stop continuous alignment)
        m_driverController.povLeft().onTrue(
            new InstantCommand(() -> {
                // If a command exists and is scheduled, cancel it; otherwise start a new one.
                if (m_aimCommand != null && m_aimCommand.isScheduled()) {
                    m_aimCommand.cancel();
                    m_robotDrive.clearRotationOverride();
                    m_aimCommand = null;
                } else {
                    m_aimCommand = new AimToAprilTag(m_robotDrive, "GENERAL_WEBCAM (1)", true);
                    m_aimCommand.schedule();
                }
            })
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
    }

    public Command getAutonomousCommand() {
        // Autonomous: only run the vision-based driving + shooter control (runs until canceled)
        return new AutoDriveToAprilTag(m_robotDrive, m_shooter, "GENERAL_WEBCAM (1)");
    }
}