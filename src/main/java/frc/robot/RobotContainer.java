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
import frc.robot.subsystems.Solenoides;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.OIConstants;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj.Timer;
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
    private final Solenoides m_solenoides = new Solenoides();
    // Control 0 = drivetrain (driver)
    private final CommandXboxController m_driverController = new CommandXboxController(0);
    // Control 1 = subsistemas (operator)
    private final CommandXboxController m_operatorController = new CommandXboxController(1);

    // Selector de autónomos (PathPlanner)
    private final SendableChooser<Command> m_autoChooser = new SendableChooser<>();

    // Selector de configuración de controles
    private enum ControlScheme {
        TWO_CONTROLLERS,
        ONE_CONTROLLER
    }

    private final SendableChooser<ControlScheme> m_controlSchemeChooser = new SendableChooser<>();

    // Debug/diagnóstico: si esto no sube a Shuffleboard, el jar nuevo NO está corriendo.
    private int m_rcHeartbeat = 0;

    // --- Angular / Intake ---
    private boolean m_intake100ToggleActive = false;
    private boolean m_intakeToggleActive = false;
    private boolean m_beltAuxToggleActive = false;

    // Angular jog (para calibración / reset 0)
    private boolean m_angularJogActive = false;

    // Debounce simple para AutoAim-tag: evita que un frame sin tag apague/prenda el shooter.
    private int m_autoAimTagPresentCycles = 0;
    private static final int kAutoAimTagDebounceCycles = 5; // ~100ms

    // Shooter fallback (equivalente al viejo "-4000 RPM").
    // Mapea a porcentaje, y el subsistema Shooter lo convertirá a VOLTAJE.
    private static final double kShooterFallbackPercent = -0.55;

    // AutoAim solo debe activarse si hay un tag válido visible.
    private boolean canEnableAutoAimNow() {
        return m_camara.hasAutoAimTag() && m_camara.getAutoAimTagId() != 0;
    }

    private void stopShooterAll() {
        m_asistedShooter.stop();
        // Apagar por completo.
        m_shooter.stop();
    }

    private void setAssistedShooterPercent(double percent) {
        m_shooter.setAssistedShooterPercent(percent);
        edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber("Shooter/Assisted/PercentCmd", percent);
    }

    public RobotContainer() {
        configurePathPlanner();
        configureControlSchemeChooser();
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
        SmartDashboard.putData("Controls/Chooser", m_controlSchemeChooser);
    }

    private void configureControlSchemeChooser() {
        // Default = 2 controles (driver + operator)
        m_controlSchemeChooser.setDefaultOption("2 controles", ControlScheme.TWO_CONTROLLERS);
        m_controlSchemeChooser.addOption("1 control", ControlScheme.ONE_CONTROLLER);
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
            // =========================
            // PathPlanner NamedCommands (event markers)
            // =========================
            // Estos nombres deben coincidir EXACTO con los markers en PathPlanner.

            // IntakeOn:
            // 1) Bajar angular al mismo punto que el botón A
            // 2) Esperar 0.5s
            // 3) Prender intake (misma dirección que usabas en LT del operador)
            NamedCommands.registerCommand(
                "IntakeOn",
                Commands.sequence(
                    new InstantCommand(() -> m_angular.irAPosicion(-(360.0 * 31.5)), m_angular),
                    Commands.waitSeconds(0.5),
                    new InstantCommand(() -> m_intake.intakeReverse(), m_intake)
                )
            );

            // IntakeOff: solo parar intake
            NamedCommands.registerCommand(
                "IntakeOff",
                new InstantCommand(() -> m_intake.stop(), m_intake)
            );

            // ShooterOn:
            // 1) Prender AutoAim
            // 2) Esperar 0.5s
            // 3) Prender shooter (assisted)
            // 4) Esperar 1.0s
            // 5) Prender bandas por 5.0s
            NamedCommands.registerCommand(
                "ShooterOn",
                Commands.sequence(
                    getAutoAimToggleCommand().onlyIf(() -> !m_robotDrive.isAutoAimEnabled()),
                    Commands.waitSeconds(0.1),
                    // Prender shooter (assisted/fallback) y sostenerlo prendido mientras corra el auto
                    // por el tiempo de este evento (luego se apaga solo).
                    new RunCommand(() -> {
                        // Reusar EXACTO la misma lógica que el RT (sin necesidad de schedule())
                        if (m_shooter.isEmergencyEnabled()) {
                            SmartDashboard.putString("AssistShooter/Auto/Mode", "EMERGENCY_STOP");
                            stopShooterAll();
                            return;
                        }

                        double cmdPercent;

                        if (!m_robotDrive.isAutoAimEnabled()) {
                            cmdPercent = m_asistedShooter.applyPercentMultiplier(kShooterFallbackPercent);
                            setAssistedShooterPercent(cmdPercent);
                            SmartDashboard.putString("AssistShooter/Auto/Mode", "AUTOAIM_OFF_FALLBACK");
                            return;
                        }

                        boolean hasTagNow = m_camara.hasAutoAimTag();
                        if (hasTagNow) {
                            m_autoAimTagPresentCycles = Math.min(kAutoAimTagDebounceCycles, m_autoAimTagPresentCycles + 1);
                        } else {
                            m_autoAimTagPresentCycles = Math.max(0, m_autoAimTagPresentCycles - 1);
                        }

                        boolean hasTagDebounced = m_autoAimTagPresentCycles >= kAutoAimTagDebounceCycles;
                        boolean tooFar = m_camara.getAutoAimDistanceM() > 5.0;
                        if (!hasTagDebounced || tooFar) {
                            cmdPercent = m_asistedShooter.applyPercentMultiplier(kShooterFallbackPercent);
                            setAssistedShooterPercent(cmdPercent);
                            SmartDashboard.putString(
                                "AssistShooter/Auto/Mode",
                                !hasTagDebounced ? "AUTOAIM_NO_TAG_FALLBACK" : "AUTOAIM_TOO_FAR_FALLBACK");
                            return;
                        }

                        if (m_asistedShooter.canShootNow()) {
                            m_shooter.clearAssistedRequest();
                            cmdPercent = m_asistedShooter.getDesiredPercent();
                            if (!Double.isFinite(cmdPercent) || Math.abs(cmdPercent) < 1e-6) {
                                cmdPercent = m_asistedShooter.applyPercentMultiplier(kShooterFallbackPercent);
                                SmartDashboard.putString("AssistShooter/Auto/Mode", "ASSIST_INVALID_FALLBACK");
                            } else {
                                SmartDashboard.putString("AssistShooter/Auto/Mode", "ASSIST_PERCENT");
                            }
                            setAssistedShooterPercent(cmdPercent);
                        } else {
                            cmdPercent = m_asistedShooter.applyPercentMultiplier(kShooterFallbackPercent);
                            setAssistedShooterPercent(cmdPercent);
                            SmartDashboard.putString("AssistShooter/Auto/Mode", "ASSIST_CANT_SHOOT_FALLBACK");
                        }
                    }, m_asistedShooter, m_shooter).withTimeout(6.5),
                    Commands.waitSeconds(1.0),
                    Commands.startEnd(
                            () -> {
                                m_shooter.startBelt();
                                m_auxMotor.startReverseAux();
                            },
                            () -> {
                                m_shooter.stopBelt();
                                m_auxMotor.stop();
                            },
                            m_shooter, m_auxMotor)
                        .withTimeout(5.0),
                    // Al terminar el evento, apagar shooter también (así siempre dura 5s de tiro total)
                    new InstantCommand(() -> {
                        m_asistedShooter.stop();
                        m_shooter.clearAssistedRequest();
                        // Volver al idle sí o sí
                        m_shooter.endShootAndReturnToIdle();
                    }, m_asistedShooter, m_shooter)
                )
            );

            // ShooterOff: apagar shooter, autoaim y bandas
            NamedCommands.registerCommand(
                "ShooterOff",
                new InstantCommand(() -> {
                    m_asistedShooter.stop();
                    m_shooter.clearAssistedRequest();
                    // NO stop total: siempre volver al idle (-0.15) con rampa
                    m_shooter.endShootAndReturnToIdle();
                    m_shooter.stopBelt();
                    m_auxMotor.stop();
                    m_robotDrive.setAutoAimEnabled(false);
                    SmartDashboard.putBoolean("AutoAim/EnabledRequested", false);
                }, m_asistedShooter, m_shooter, m_auxMotor, m_robotDrive)
            );

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
        ControlScheme scheme = m_controlSchemeChooser.getSelected();
        if (scheme == null) {
            scheme = ControlScheme.TWO_CONTROLLERS;
        }
        SmartDashboard.putString("Controls/Selected", scheme == ControlScheme.ONE_CONTROLLER ? "1 control" : "2 controles");

        if (scheme == ControlScheme.ONE_CONTROLLER) {
            configureBindingsOneController();
        } else {
            configureBindingsTwoControllers();
        }
    }

    private void configureBindingsTwoControllers() {
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

        // LT del Driver: LIBRE (shooter ahora es solo del operador)

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
                    // Prender 100%
                    // Nota: la dirección real depende de cómo esté implementado el Intake.
                    // Aquí llamamos a intakeFullReverse() tal como está en el subsistema.
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

        // Angular: Y -> posición "home" (setpoint = -360°)
        m_operatorController.y().onTrue(
            new InstantCommand(() -> m_angular.irAPosicion((-360*3)), m_angular)
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
            new InstantCommand(() -> m_angular.irAPosicion(-(360.0 * 31.5)), m_angular)
        );

        // Angular: START -> posición intermedia
        m_operatorController.start().onTrue(
            new InstantCommand(() -> m_angular.irAPosicion(-(360.0 * 17)), m_angular)
        );

        // POVLeft (tap): Solenoides -> alternar ambos (A y B)
        m_operatorController.povLeft().onTrue(
            new InstantCommand(() -> m_solenoides.alternarAmbos(), m_solenoides)
        );

        // Driver POVLeft (one-shot): Toggle AutoAim ON/OFF
        // Reglas:
        // - Para prender AutoAim: debe haber tag válido visible (Camara.hasAutoAimTag()).
        // - Para apagarlo: siempre se permite.
        m_driverController.povLeft().onTrue(
            getAutoAimToggleCommand()
        );

        // Shooter Assisted (RT) en el operador
        m_operatorController.rightTrigger().whileTrue(
            getAssistedShooterCommandLoop()
        ).onFalse(
            getAssistedShooterReleaseCommand()
        );

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

        // POVDown (tap): girar 180° (media vuelta) - mismo comportamiento que en 1 control
        m_operatorController.povDown().onTrue(
            Commands.sequence(
                new InstantCommand(() -> SmartDashboard.putString("Drive/Turn180/State", "START")),
                // Pequeño pulso de giro. Ajusta Drive/Turn180/RotCmd y Drive/Turn180/TimeS desde Shuffleboard.
                Commands.runOnce(() -> {
                    double rotCmd = SmartDashboard.getNumber("Drive/Turn180/RotCmd", 0.85);
                    double timeS = SmartDashboard.getNumber("Drive/Turn180/TimeS", 0.75);
                    rotCmd = MathUtil.clamp(rotCmd, -1.0, 1.0);
                    timeS = MathUtil.clamp(timeS, 0.1, 2.0);
                    SmartDashboard.putNumber("Drive/Turn180/RotCmdApplied", rotCmd);
                    SmartDashboard.putNumber("Drive/Turn180/TimeSApplied", timeS);
                    SmartDashboard.putNumber("Drive/Turn180/T0", Timer.getFPGATimestamp());
                }),
                Commands.run(() -> {
                        double rotCmd = SmartDashboard.getNumber("Drive/Turn180/RotCmdApplied", 0.85);
                        m_robotDrive.setRotationOverride(rotCmd);
                        SmartDashboard.putString("Drive/Turn180/State", "TURNING");
                    }, m_robotDrive)
                    .withTimeout(SmartDashboard.getNumber("Drive/Turn180/TimeSApplied", 0.75)),
                new InstantCommand(() -> {
                    m_robotDrive.clearRotationOverride();
                    SmartDashboard.putString("Drive/Turn180/State", "DONE");
                }, m_robotDrive)
            )
        );

        // PovDown (while held): Banda + AuxMotor
        m_operatorController.povDown().whileTrue(
            new RunCommand(() -> {
                m_shooter.reverseBelt();
                m_auxMotor.stop();
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

        // Nota: el toggle de AutoAim se movió al driver (POVLeft).
    }

    private void configureBindingsOneController() {
        // =========================
        // 1 Control (Driver) - TODO en el driver
        // =========================

        // RB: freno en seco (setX)
        m_driverController.rightBumper().whileTrue(
            new RunCommand(() -> m_robotDrive.setX(), m_robotDrive)
        );

        // POVLeft: Toggle AutoAim
        m_driverController.povLeft().onTrue(
            getAutoAimToggleCommand()
        );

        // RT: Shooter
        m_driverController.rightTrigger().whileTrue(
            getAssistedShooterCommandLoop()
        ).onFalse(
            getAssistedShooterReleaseCommand()
        );

        // LT: Intake toggle (nota: en este robot los nombres forward/reverse están invertidos respecto a física;
        // mantenemos lo que ya funcionaba: LT en operador llamaba intakeReverse())
        m_driverController.leftTrigger().onTrue(
            new InstantCommand(() -> {
                if (m_intake100ToggleActive) {
                    return;
                }
                m_intakeToggleActive = !m_intakeToggleActive;
                if (m_intakeToggleActive) {
                    m_intake.intakeReverse();
                } else {
                    m_intake.stop();
                }
            }, m_intake)
        );

        // LB: Intake para atrás
        m_driverController.leftBumper().whileTrue(
            new RunCommand(() -> m_intake.intakeForward(), m_intake)
        ).onFalse(
            new InstantCommand(() -> m_intake.stop(), m_intake)
        );

        // X: bandas + auxiliar (toggle)
        m_driverController.x().onTrue(
            new InstantCommand(() -> {
                m_beltAuxToggleActive = !m_beltAuxToggleActive;
                if (m_beltAuxToggleActive) {
                    m_shooter.startBelt();
                    m_auxMotor.startReverseAux();
                } else {
                    m_shooter.stopBelt();
                    m_auxMotor.stop();
                }
            }, m_shooter, m_auxMotor)
        );

        // Y: Angular home
        m_driverController.y().onTrue(
            new InstantCommand(() -> m_angular.irAPosicion((-360 * 3)), m_angular)
        );

        // A: Angular abajo
        m_driverController.a().onTrue(
            new InstantCommand(() -> m_angular.irAPosicion(-(360.0 * 31.5)), m_angular)
        );

        // B: ciclo anti-atoradas
        m_driverController.b().onTrue(
            new InstantCommand(() -> m_angular.setUnjamCycleEnabled(true), m_angular)
        );

        // POVRight (while held): subir Angular manual (open-loop) para calibración
        m_driverController.povRight().whileTrue(
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

        // START: set 0 del Angular
        m_driverController.start().onTrue(
            new InstantCommand(() -> m_angular.resetEncoder(), m_angular)
        );

        // BACK: set 0 de la IMU (swerves)
        m_driverController.back().onTrue(
            new InstantCommand(() -> m_robotDrive.zeroHeading(), m_robotDrive)
        );

        // POVDown (tap): girar 180° (media vuelta)
        // Implementación simple/robusta: aplicar una rotación fija por un tiempo y luego soltar.
        // (Si quieres absoluto por heading con PID, lo hacemos después.)
        m_driverController.povDown().onTrue(
            Commands.sequence(
                new InstantCommand(() -> SmartDashboard.putString("Drive/Turn180/State", "START")),
                // Pequeño pulso de giro. Ajusta Drive/Turn180/RotCmd y Drive/Turn180/TimeS desde Shuffleboard.
                Commands.runOnce(() -> {
                    double rotCmd = SmartDashboard.getNumber("Drive/Turn180/RotCmd", 0.85);
                    double timeS = SmartDashboard.getNumber("Drive/Turn180/TimeS", 0.75);
                    rotCmd = MathUtil.clamp(rotCmd, -1.0, 1.0);
                    timeS = MathUtil.clamp(timeS, 0.1, 2.0);
                    SmartDashboard.putNumber("Drive/Turn180/RotCmdApplied", rotCmd);
                    SmartDashboard.putNumber("Drive/Turn180/TimeSApplied", timeS);
                    SmartDashboard.putNumber("Drive/Turn180/T0", Timer.getFPGATimestamp());
                }),
                Commands.run(() -> {
                        double rotCmd = SmartDashboard.getNumber("Drive/Turn180/RotCmdApplied", 0.85);
                        m_robotDrive.setRotationOverride(rotCmd);
                        SmartDashboard.putString("Drive/Turn180/State", "TURNING");
                    }, m_robotDrive)
                    .withTimeout(SmartDashboard.getNumber("Drive/Turn180/TimeSApplied", 0.75)),
                new InstantCommand(() -> {
                    m_robotDrive.clearRotationOverride();
                    SmartDashboard.putString("Drive/Turn180/State", "DONE");
                }, m_robotDrive)
            )
        );
    }

    private Command getAutoAimToggleCommand() {
        return new InstantCommand(() -> {
            boolean wantEnable = !m_robotDrive.isAutoAimEnabled();
            if (wantEnable && !canEnableAutoAimNow()) {
                // Si no hay un tag confiable, no habilitar AutoAim.
                m_robotDrive.setAutoAimEnabled(false);
                SmartDashboard.putBoolean("AutoAim/EnabledRequested", false);
                SmartDashboard.putString("AutoAim/EnableReject", "No valid tag");
                return;
            }
            m_robotDrive.setAutoAimEnabled(wantEnable);
            SmartDashboard.putBoolean("AutoAim/EnabledRequested", wantEnable);
            SmartDashboard.putString("AutoAim/EnableReject", "");
        }, m_robotDrive);
    }

    private Command getAssistedShooterCommandLoop() {
        return new RunCommand(
            () -> {
                if (m_shooter.isEmergencyEnabled()) {
                    SmartDashboard.putString("AssistShooter/RT/Mode", "EMERGENCY_STOP");
                    stopShooterAll();
                    return;
                }

                // Elegir UN percent a mandar este ciclo. No hagas stop+set en el mismo loop.
                double cmdPercent;

                // Reglas (porcentaje->voltaje):
                // 1) AutoAim OFF  -> shooter fijo (fallback)
                // 2) AutoAim ON pero SIN tag o >5m -> shooter fijo (fallback)
                // 3) AutoAim ON y tag válido (<=5m) -> AssistedShooter (fórmula -> %)
                if (!m_robotDrive.isAutoAimEnabled()) {
                    cmdPercent = m_asistedShooter.applyPercentMultiplier(kShooterFallbackPercent);
                    setAssistedShooterPercent(cmdPercent);
                    SmartDashboard.putString("AssistShooter/RT/Mode", "AUTOAIM_OFF_FALLBACK");
                    return;
                }

                // AutoAim ON
                boolean hasTagNow = m_camara.hasAutoAimTag();
                if (hasTagNow) {
                    m_autoAimTagPresentCycles = Math.min(kAutoAimTagDebounceCycles, m_autoAimTagPresentCycles + 1);
                } else {
                    m_autoAimTagPresentCycles = Math.max(0, m_autoAimTagPresentCycles - 1);
                }

                boolean hasTagDebounced = m_autoAimTagPresentCycles >= kAutoAimTagDebounceCycles;
                boolean tooFar = m_camara.getAutoAimDistanceM() > 5.0;
                if (!hasTagDebounced || tooFar) {
                    cmdPercent = m_asistedShooter.applyPercentMultiplier(kShooterFallbackPercent);
                    setAssistedShooterPercent(cmdPercent);
                    SmartDashboard.putString(
                        "AssistShooter/RT/Mode",
                        !hasTagDebounced ? "AUTOAIM_NO_TAG_FALLBACK" : "AUTOAIM_TOO_FAR_FALLBACK");
                    return;
                }

                // AutoAim ON + tag dentro de 5m
                if (m_asistedShooter.canShootNow()) {
                    // NO mandar stop aqui: eso pelea con el assisted y puede hacer que caiga a 0 por la rampa.
                    // Solo aseguramos que no este latcheado un modo anterior.
                    m_shooter.clearAssistedRequest();
                    cmdPercent = m_asistedShooter.getDesiredPercent();
                    // Seguridad: si por alguna razon sale 0 o NaN, usa fallback.
                    if (!Double.isFinite(cmdPercent) || Math.abs(cmdPercent) < 1e-6) {
                        cmdPercent = m_asistedShooter.applyPercentMultiplier(kShooterFallbackPercent);
                        SmartDashboard.putString("AssistShooter/RT/Mode", "ASSIST_INVALID_FALLBACK");
                    } else {
                        SmartDashboard.putString("AssistShooter/RT/Mode", "ASSIST_PERCENT");
                    }
                    setAssistedShooterPercent(cmdPercent);
                } else {
                    // Si por alguna razón canShootNow() no deja (distancia 0, etc.), usar el fallback fuerte
                    cmdPercent = m_asistedShooter.applyPercentMultiplier(kShooterFallbackPercent);
                    setAssistedShooterPercent(cmdPercent);
                    SmartDashboard.putString("AssistShooter/RT/Mode", "ASSIST_CANT_SHOOT_FALLBACK");
                }

                SmartDashboard.putNumber("AssistShooter/RT/CmdPercent", cmdPercent);
                SmartDashboard.putBoolean("AssistShooter/RT/AutoAimEnabled", m_robotDrive.isAutoAimEnabled());
                SmartDashboard.putBoolean("AssistShooter/RT/HasTag", m_camara.hasAutoAimTag());
                SmartDashboard.putNumber("AssistShooter/RT/DistanceM", m_camara.getAutoAimDistanceM());
            },
            m_asistedShooter, m_shooter);
    }

    private Command getAssistedShooterReleaseCommand() {
        return new InstantCommand(() -> {
            // Al soltar RT: NO forzar un stop total.
            // Dejamos que el "idle spin" del subsistema Shooter se encargue de mantenerlo girando
            // (o apagarlo si el idle está deshabilitado / HardStop está activo).
            m_asistedShooter.stop();
            // Importante: quita el estado de assisted para que no se quede "pegado".
            m_shooter.clearAssistedRequest();
            m_shooter.endShootAndReturnToIdle();
            m_shooter.stopBelt();
            m_auxMotor.stop();
        }, m_asistedShooter, m_shooter, m_auxMotor);
    }

    public Command getAutonomousCommand() {
        return m_autoChooser.getSelected();
    }

}