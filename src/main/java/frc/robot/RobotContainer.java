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
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import java.nio.file.DirectoryStream;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;

public class RobotContainer {
    private static final double kDriveScale = 1.0;

    private final Camara m_camara = new Camara();
    private final DriveSubsystem m_robotDrive = new DriveSubsystem(m_camara);
    private final Angular m_angular = new Angular();
    private final Intake m_intake = new Intake();
    private final Shooter m_shooter = new Shooter();
    private final AsistedShooter m_asistedShooter = new AsistedShooter(m_camara);
    private final AuxMotor m_auxMotor = new AuxMotor();
    // Control 0 = drivetrain (driver)
    private final CommandXboxController m_driverController = new CommandXboxController(0);
    // Control 1 = subsistemas (operator)
    private final CommandXboxController m_operatorController = new CommandXboxController(1);

    // Selector de autónomos (PathPlanner)
    private final SendableChooser<Command> m_autoChooser = new SendableChooser<>();

    // Debug/diagnóstico: si esto no sube a Shuffleboard, el jar nuevo NO está corriendo.
    private int m_rcHeartbeat = 0;

    // --- Angular / Intake ---
    private boolean m_intake100ToggleActive = false;

    // Angular jog (para calibración / reset 0)
    private boolean m_angularJogActive = false;

    private void stopShooterAll() {
        m_asistedShooter.stop();
        m_shooter.stopManualShooter();
    }

    private void setAssistedShooterPercent(double percent) {
        // Rampa para que el shooter no cambie de golpe cuando cambia el setpoint.
        // (reduce el "pulsing" cuando el tag se pierde/regresa)
        double ramped = m_shooterPercentLimiter.calculate(percent);
        m_shooter.setAssistedShooterPercent(ramped);
        edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber("Shooter/Assisted/PercentCmd", percent);
        edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber("Shooter/Assisted/PercentRamped", ramped);
    }

    // Shooter anti-chatter: rampa de setpoint.
    private final edu.wpi.first.math.filter.SlewRateLimiter m_shooterPercentLimiter =
        new edu.wpi.first.math.filter.SlewRateLimiter(2.0);

    public RobotContainer() {
        configurePathPlanner();
        configureBindings();

        m_robotDrive.setDefaultCommand(
            new RunCommand(
                () -> {
                    double leftY = -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband) * kDriveScale;
                    double leftX = -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband) * kDriveScale;
                    double rot = -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband) * kDriveScale;

                    // AutoAim: limitar velocidad máxima de traslación para facilitar apuntado.
                    // (0.5 = 50% de la velocidad que normalmente permitiría el joystick)
                    if (m_robotDrive.isAutoAimEnabled()) {
                        leftY *= 0.5;
                        leftX *= 0.5;
                    }

                    double rotWithAutoAim = m_robotDrive.getRotationForDrive(rot);
                    m_robotDrive.drive(
                        leftY,
                        leftX,
                        rotWithAutoAim,
                        true);
                },
                m_robotDrive));

        // Publicar chooser al dashboard
        SmartDashboard.putData("Auto/Chooser", m_autoChooser);
    }

    /**
     * Llamar esto desde Robot.robotPeriodic() si quieres un heartbeat constante.
     * (Si no puedes/quieres tocar Robot.java, igual publicamos bastante dentro de configurePathPlanner)
     */
    public void publishRobotContainerHeartbeat() {
        SmartDashboard.putNumber("RC/Heartbeat", m_rcHeartbeat++);
    }

    private void configurePathPlanner() {
        SmartDashboard.putString("Auto/Status", "configuring");
        try {
            // Validación previa: si al JSON le faltan keys, PathPlanner truena con NPE/Null.
            validatePPSettingsJson();

            // RobotConfig se carga desde la GUI de PathPlanner (Robot Settings).
            // Si no existe aún, la excepción te avisa y el robot sigue funcionando sin autos.
            RobotConfig config = RobotConfig.fromGUISettings();

            AutoBuilder.configure(
                m_robotDrive::getPose,
                m_robotDrive::resetOdometry,
                m_robotDrive::getRobotRelativeSpeeds,
                (ChassisSpeeds speeds) -> m_robotDrive.driveRobotRelative(speeds),
                new PPHolonomicDriveController(
                    new PIDConstants(5.0, 0.0, 0.0),
                    new PIDConstants(5.0, 0.0, 0.0)),
                config,
                () -> DriverStation.getAlliance().isPresent()
                    && DriverStation.getAlliance().get() == DriverStation.Alliance.Red,
                m_robotDrive
            );

            // Autodiscover: usar el deploy dir real de WPILib (en roboRIO = /home/lvuser/deploy)
            // para evitar bugs por paths relativos o por comportamiento de java.io.File.
            Path autosDir = Filesystem.getDeployDirectory().toPath().resolve("pathplanner").resolve("autos");
            SmartDashboard.putString("Auto/AutoDir", autosDir.toString());

            m_autoChooser.setDefaultOption("DoNothing", new InstantCommand());
            List<String> autos = new ArrayList<>();

            if (Files.exists(autosDir) && Files.isDirectory(autosDir)) {
                try (DirectoryStream<Path> stream = Files.newDirectoryStream(autosDir, "*.auto")) {
                    for (Path p : stream) {
                        String fileName = p.getFileName().toString();
                        String base = fileName.substring(0, fileName.length() - ".auto".length());
                        autos.add(base);
                    }
                }
            }

            Collections.sort(autos);
            int added = 0;
            StringBuilder found = new StringBuilder();
            for (String base : autos) {
                try {
                    m_autoChooser.addOption(base, AutoBuilder.buildAuto(base));
                    added++;
                    if (found.length() > 0) found.append(", ");
                    found.append(base);
                } catch (Exception ex) {
                    SmartDashboard.putString("Auto/BuildError/" + base, ex.toString());
                }
            }

            SmartDashboard.putNumber("Auto/AutoCount", added);
            SmartDashboard.putString("Auto/AutosFound", found.toString());
            SmartDashboard.putString("Auto/Status", "ok");

            // Extra: si no se encontró ninguno, deja un mensaje bien claro.
            if (added == 0) {
                SmartDashboard.putString("Auto/Status", "no .auto found in deploy/pathplanner/autos");
            }
        } catch (Exception e) {
            // Si falta RobotConfig/GUI settings, dejamos el chooser mínimo para no romper.
            m_autoChooser.setDefaultOption("DoNothing", new InstantCommand());
            SmartDashboard.putString("Auto/PathPlannerError", String.valueOf(e));
            SmartDashboard.putString("Auto/PathPlannerErrorType", e.getClass().getName());
            SmartDashboard.putString("Auto/PathPlannerErrorMsg", String.valueOf(e.getMessage()));
            SmartDashboard.putString("Auto/PathPlannerErrorTop", getTopStackTrace(e, 6));
            DriverStation.reportError("PathPlanner configure failed: " + e, e.getStackTrace());
            SmartDashboard.putString("Auto/Status", "error");

            // FALLBACK: aunque el config falle, al menos listar autos por nombre.
            tryPopulateChooserNamesOnly(String.valueOf(e));
        }
    }

    private void validatePPSettingsJson() {
        try {
            Path settingsPath = Filesystem.getDeployDirectory().toPath().resolve("pathplanner").resolve("settings.json");
            SmartDashboard.putString("Auto/SettingsPath", settingsPath.toString());
            if (!Files.exists(settingsPath)) {
                SmartDashboard.putString("Auto/SettingsValidation", "MISSING settings.json");
                return;
            }

            String content = Files.readString(settingsPath);
            JSONObject json = (JSONObject) new JSONParser().parse(content);

            // Keys requeridas por RobotConfig.fromGUISettings() (PathPlannerLib 2026.1.2)
            String[] required = new String[] {
                "holonomicMode",
                "robotMass",
                "robotMOI",
                "driveWheelRadius",
                "driveGearing",
                "maxDriveSpeed",
                "wheelCOF",
                "driveMotorType",
                "driveCurrentLimit",
            };

            StringBuilder missing = new StringBuilder();
            for (String k : required) {
                if (!json.containsKey(k) || json.get(k) == null) {
                    if (missing.length() > 0) missing.append(", ");
                    missing.append(k);
                }
            }

            boolean holonomic = Boolean.TRUE.equals(json.get("holonomicMode"));
            if (holonomic) {
                String[] modules = new String[] {
                    "flModuleX","flModuleY","frModuleX","frModuleY","blModuleX","blModuleY","brModuleX","brModuleY"
                };
                for (String k : modules) {
                    if (!json.containsKey(k) || json.get(k) == null) {
                        if (missing.length() > 0) missing.append(", ");
                        missing.append(k);
                    }
                }
            } else {
                String k = "robotTrackwidth";
                if (!json.containsKey(k) || json.get(k) == null) {
                    if (missing.length() > 0) missing.append(", ");
                    missing.append(k);
                }
            }

            if (missing.length() == 0) {
                SmartDashboard.putString("Auto/SettingsValidation", "OK");
            } else {
                SmartDashboard.putString("Auto/SettingsValidation", "Missing keys: " + missing);
            }

            // Dump mínimo (para comprobar que está leyendo el archivo correcto)
            SmartDashboard.putString("Auto/SettingsDriveMotorType", String.valueOf(json.get("driveMotorType")));
        } catch (Exception ex) {
            SmartDashboard.putString("Auto/SettingsValidation", "ERROR parsing settings.json: " + ex);
        }
    }

    private void tryPopulateChooserNamesOnly(String error) {
        try {
            Path autosDir = Filesystem.getDeployDirectory().toPath().resolve("pathplanner").resolve("autos");
            SmartDashboard.putString("Auto/FallbackAutoDir", autosDir.toString());

            List<String> autos = new ArrayList<>();
            if (Files.exists(autosDir) && Files.isDirectory(autosDir)) {
                try (DirectoryStream<Path> stream = Files.newDirectoryStream(autosDir, "*.auto")) {
                    for (Path p : stream) {
                        String fileName = p.getFileName().toString();
                        String base = fileName.substring(0, fileName.length() - ".auto".length());
                        autos.add(base);
                    }
                }
            }
            Collections.sort(autos);

            int added = 0;
            for (String base : autos) {
                // Comando placeholder: para que el chooser muestre opciones aunque la config esté mal.
                m_autoChooser.addOption(
                    base,
                    new InstantCommand(() -> DriverStation.reportError(
                        "Auto '" + base + "' selected but PathPlanner is not configured. Error: " + error,
                        false))
                );
                added++;
            }

            SmartDashboard.putNumber("Auto/FallbackAutoCount", added);
            if (added > 0) {
                SmartDashboard.putString("Auto/Status", "error (fallback chooser names only)");
            }
        } catch (Exception ex) {
            SmartDashboard.putString("Auto/FallbackError", String.valueOf(ex));
        }
    }

    private static String getTopStackTrace(Throwable t, int maxFrames) {
        if (t == null) return "";
        StackTraceElement[] st = t.getStackTrace();
        if (st == null || st.length == 0) return "";
        int n = Math.min(maxFrames, st.length);
        StringBuilder sb = new StringBuilder();
        for (int i = 0; i < n; i++) {
            if (i > 0) sb.append(" \n");
            sb.append(st[i].toString());
        }
        return sb.toString();
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

        // LT (while held): Shooter (manual/assisted) para Control 0
        m_driverController.leftTrigger().whileTrue(
            new RunCommand(
                () -> {
                    if (m_shooter.isEmergencyEnabled()) {
                        stopShooterAll();
                        return;
                    }

                    // Reglas:
                    // 1) AutoAim OFF  -> shooter fijo -0.7
                    // 2) AutoAim ON pero SIN tag o >4m -> shooter fijo -0.8
                    // 3) AutoAim ON y tag válido (<=4m) -> AssistedShooter (fórmula)
                    if (!m_robotDrive.isAutoAimEnabled()) {
                        stopShooterAll();
                        setAssistedShooterPercent(-0.7);
                        return;
                    }

                    // AutoAim ON
                    boolean hasTagNow = m_camara.hasAutoAimTag();
                    edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putBoolean("Shooter/AutoAim/HasTagNow", hasTagNow);
                    if (!hasTagNow || m_camara.getAutoAimDistanceM() > 4.0) {
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
            new InstantCommand(() -> m_angular.irAPosicion((-360)), m_angular)
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
            new InstantCommand(() -> m_angular.irAPosicion(-(360.0 * 30
            )), m_angular)
        );

        // Angular: START -> posición intermedia
        m_operatorController.start().onTrue(
            new InstantCommand(() -> m_angular.irAPosicion(-(360.0 * 17)), m_angular)
        );

        m_operatorController.rightTrigger().whileTrue(
            new RunCommand(
                () -> {
                    if (m_shooter.isEmergencyEnabled()) {
                        stopShooterAll();
                        return;
                    }

                    // Reglas:
                    // 1) AutoAim OFF  -> shooter fijo -0.7
                    // 2) AutoAim ON pero SIN tag o >4m -> shooter fijo -0.8
                    // 3) AutoAim ON y tag válido (<=4m) -> AssistedShooter (fórmula)
                    if (!m_robotDrive.isAutoAimEnabled()) {
                        stopShooterAll();
                        setAssistedShooterPercent(-0.7);
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
                m_angular.holdCurrentPosition();
            }, m_angular)
        );

        // Back button: tomar esta posición actual como 0 del Angular (CAN 54)
        // Nota: 'back()' existe en CommandXboxController para Xbox (View/Back).
        m_operatorController.back().onTrue(
            new InstantCommand(() -> m_angular.resetEncoder(), m_angular)
        );
    }

    public Command getAutonomousCommand() {
        return m_autoChooser.getSelected();
    }

}