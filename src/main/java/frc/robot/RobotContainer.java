package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Camara;
import frc.robot.subsystems.Angular;
import frc.robot.subsystems.Align;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.AuxMotor;
import frc.robot.subsystems.AsistedShooter;
import frc.robot.subsystems.Solenoides;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.OIConstants;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import java.nio.file.DirectoryStream;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;

public class RobotContainer {

    private static final double kDriveScale = 1.0;

    private final Camara m_camara = new Camara();
    private final DriveSubsystem m_robotDrive = new DriveSubsystem(m_camara);
    private final Angular m_angular = new Angular();
    private final Command m_alignCommand = Align.createOneShotCommand(m_robotDrive, m_camara);
    private final Intake m_intake = new Intake();
    private final Shooter m_shooter = new Shooter();
    private final AsistedShooter m_asistedShooter = new AsistedShooter(m_camara);
    private final AuxMotor m_auxMotor = new AuxMotor();
    private final Solenoides m_solenoides = new Solenoides();

    private final DigitalOutput Estado_1 = new DigitalOutput(6);
    private final DigitalOutput Estado_2 = new DigitalOutput(7);
    private final DigitalOutput Estado_3 = new DigitalOutput(8);
    private final DigitalOutput Estado_4 = new DigitalOutput(9);

    // Estado LED (solo uno encendido a la vez)
    private DigitalOutput m_activeStatusDio = null;

    private final CommandXboxController m_driverController =
        new CommandXboxController(OIConstants.kDriverControllerPort);
    private final CommandXboxController m_operatorController = new CommandXboxController(1);

    private SendableChooser<Command> m_autoChooser = new SendableChooser<>();
    private final Map<String, Command> m_autoOptions = new LinkedHashMap<>();
    private boolean m_autoConfigured = false;

    private int m_rcHeartbeat = 0;

    private boolean m_intake100ToggleActive = false;
    private boolean m_angularOscillateActive = false;
    private boolean m_angularOscillateForward = true;
    private double m_angularOscillateHomeDeg = 0.0;
    private double m_angularOscillateBackDeg = 0.0;
    private static final double kAngularHomeDeg = 0.0;
    private static final double kAngularOscillateTolDeg = 15.0;
    private static final double kAngularOscillateDeltaDeg = 360.0 * 5.0;
    private final Command m_angularOscillateCommand = new RunCommand(() -> {
        double target = m_angularOscillateForward ? m_angularOscillateHomeDeg : m_angularOscillateBackDeg;
        m_angular.irAPosicion(target);
        if (Math.abs(m_angular.getPositionDeg() - target) <= kAngularOscillateTolDeg) {
            m_angularOscillateForward = !m_angularOscillateForward;
        }
    }, m_angular);


    // Angular jog (para calibración / reset 0)
    private boolean m_angularJogActive = false;

    // Auto command helpers (PathPlanner): shooter toggle en auto
    private Command m_autoShooterHoldCommand = null;

    // Debounce simple para AutoAim-tag: evita que un frame sin tag apague/prenda el shooter.
    private int m_autoAimTagPresentCycles = 0;
    private static final int kAutoAimTagDebounceCycles = 5; // ~100ms

    // Shooter fallback (equivalente al viejo "-4000 RPM").
    // Mapea a porcentaje, y el subsistema Shooter lo convertirá a VOLTAJE.
    //AQUI ES EL SHOOTER MANUAL
    private static final double kShooterFallbackPercent = -0.68;
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

    private void logAutoEvent(String name) {
        SmartDashboard.putString("Auto/Event", name);
        DriverStation.reportWarning("Auto event: " + name, false);
    }

    public RobotContainer() {
        configurePathPlanner();
        configureBindingsTwoControllers();
        Align.publishDefaults();


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

        // Publicar choosers al dashboard
        SmartDashboard.putData("Auto/Chooser", m_autoChooser);
    }

    /**
     * Llamar esto desde Robot.robotPeriodic() si quieres un heartbeat constante.
     * (Si no puedes/quieres tocar Robot.java, igual publicamos bastante dentro de configurePathPlanner)
     */
    public void publishRobotContainerHeartbeat() {
        SmartDashboard.putNumber("RC/Heartbeat", m_rcHeartbeat++);
    }

    /**
     * Actualiza los DIO para LEDs de estado.
     * Regla: solo un DIO encendido a la vez (si ninguno aplica, todos apagados).
     * Prioridad: AutoAim > Emergency shooter > Assisted shooter > Intake 100% > Oscilación Angular.
     */
    public void updateStatusDios() {
        DigitalOutput desired = null;

        if (m_robotDrive.isAutoAimEnabled()) {
            desired = Estado_1; // AutoAim = DIO 6
        } else if (m_shooter.isEmergencyEnabled()) {
            desired = Estado_4; // Emergency = DIO 9
        } else if (m_shooter.isAssistedActive()) {
            desired = Estado_2; // Assisted shooter = DIO 7
        } else if (m_intake100ToggleActive) {
            desired = Estado_3; // Intake 100% = DIO 8
        } else if (m_angularOscillateActive) {
            desired = Estado_4; // Oscilación angular = DIO 9 (solo si no hubo emergencia)
        }

        if (desired != m_activeStatusDio) {
            Estado_1.set(desired == Estado_1);
            Estado_2.set(desired == Estado_2);
            Estado_3.set(desired == Estado_3);
            Estado_4.set(desired == Estado_4);
            m_activeStatusDio = desired;
        }
    }

    private List<String> readAutoBaseNames() {
        List<String> autos = new ArrayList<>();
        Path autosDir = Filesystem.getDeployDirectory().toPath().resolve("pathplanner").resolve("autos");
        if (Files.exists(autosDir) && Files.isDirectory(autosDir)) {
            try (DirectoryStream<Path> stream = Files.newDirectoryStream(autosDir, "*.auto")) {
                for (Path p : stream) {
                    String fileName = p.getFileName().toString();
                    String base = fileName.substring(0, fileName.length() - ".auto".length());
                    autos.add(base);
                }
            } catch (Exception ex) {
                DriverStation.reportError("Auto read failed: " + ex, ex.getStackTrace());
            }
        }
        return autos;
    }

    private void rebuildAutoChooser() {
        // Autodiscover: usar el deploy dir real de WPILib (en roboRIO = /home/lvuser/deploy)
        // para evitar bugs por paths relativos o por comportamiento de java.io.File.
    m_autoChooser = new SendableChooser<>();
    m_autoOptions.clear();

        List<String> autos = readAutoBaseNames();

        Collections.sort(autos);

        if (autos.size() == 1) {
            // Si solo hay un auto, hacerlo default para evitar que se quede en DoNothing.
            Command cmd = AutoBuilder.buildAuto(autos.get(0));
            m_autoChooser.setDefaultOption(autos.get(0), cmd);
            m_autoOptions.put(autos.get(0), cmd);
        } else {
            Command cmd = new InstantCommand();
            m_autoChooser.setDefaultOption("DoNothing", cmd);
            m_autoOptions.put("DoNothing", cmd);
        }
        for (String base : autos) {
            try {
                if (autos.size() != 1 || !base.equals(autos.get(0))) {
                    Command cmd;
                    if ("30Derecha".equals(base)) {
                        if (!m_autoConfigured) {
                            cmd = build30DerechaFallback();
                            DriverStation.reportWarning("AutoBuilder not configured; using open-loop fallback for 30Derecha", false);
                        } else {
                            cmd = AutoBuilder.buildAuto(base);
                        }
                    } else {
                        cmd = AutoBuilder.buildAuto(base);
                    }
                    m_autoChooser.addOption(base, cmd);
                    m_autoOptions.put(base, cmd);
                }
            } catch (Exception ex) {
                DriverStation.reportError("Auto build failed for " + base + ": " + ex, ex.getStackTrace());
                if ("30Derecha".equals(base)) {
                    Command fallback = build30DerechaFallback();
                    m_autoChooser.addOption(base, fallback);
                    m_autoOptions.put(base, fallback);
                    DriverStation.reportWarning("Using fallback auto for 30Derecha", false);
                }
            }
        }

        SmartDashboard.putData("Auto/Chooser", m_autoChooser);
    }

    private void configurePathPlanner() {
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
                    new InstantCommand(() -> {
                        logAutoEvent("IntakeOn");
                        m_angular.irAPosicion(-(360.0 * 31.5));
                    }, m_angular),
                    Commands.waitSeconds(0.5),
                    new InstantCommand(() -> m_intake.intakeReverse(), m_intake)
                )
            );

            // IntakeOff: solo parar intake
            NamedCommands.registerCommand(
                "IntakeOff",
                new InstantCommand(() -> {
                    logAutoEvent("IntakeOff");
                    m_intake.stop();
                }, m_intake)
            );

            // ===== Nombres nuevos para autos (lowercase) =====
            // angularAbajo: mismo que botón A del operador
            NamedCommands.registerCommand(
                "angularAbajo",
                new InstantCommand(() -> {
                    logAutoEvent("angularAbajo");
                    m_angular.irAPosicion(-(360.0 * 31.5));
                }, m_angular)
            );

            // angularArriba: mismo que botón Y del operador (home)
            NamedCommands.registerCommand(
                "angularArriba",
                new InstantCommand(() -> {
                    logAutoEvent("angularArriba");
                    m_angular.irAPosicion((-360 * 3));
                }, m_angular)
            );

            // intakeOn: mismo que LT del operador (mantener)
            NamedCommands.registerCommand(
                "intakeOn",
                new InstantCommand(() -> {
                    logAutoEvent("intakeOn");
                    m_intake.intakeReverse();
                }, m_intake)
            );

            // intakeOff: apagar intake
            NamedCommands.registerCommand(
                "intakeOff",
                new InstantCommand(() -> {
                    logAutoEvent("intakeOff");
                    m_intake.stop();
                }, m_intake)
            );

            // autoAimEnable: activar AutoAim (solo si hay tag válido)
            NamedCommands.registerCommand(
                "autoAimEnable",
                new InstantCommand(() -> {
                    logAutoEvent("autoAimEnable");
                    if (!canEnableAutoAimNow()) {
                        m_robotDrive.setAutoAimEnabled(false);
                        SmartDashboard.putBoolean("AutoAim/EnabledRequested", false);
                        SmartDashboard.putString("AutoAim/EnableReject", "No valid tag");
                        return;
                    }
                    m_robotDrive.setAutoAimEnabled(true);
                    SmartDashboard.putBoolean("AutoAim/EnabledRequested", true);
                    SmartDashboard.putString("AutoAim/EnableReject", "");
                }, m_robotDrive)
            );

            // autoAimDisable: desactivar AutoAim
            NamedCommands.registerCommand(
                "autoAimDisable",
                new InstantCommand(() -> {
                    logAutoEvent("autoAimDisable");
                    m_robotDrive.setAutoAimEnabled(false);
                    SmartDashboard.putBoolean("AutoAim/EnabledRequested", false);
                    SmartDashboard.putString("AutoAim/EnableReject", "");
                }, m_robotDrive)
            );

            // shooterOn: equivalente a dejar RT presionado (toggle)
            NamedCommands.registerCommand(
                "shooterOn",
                new InstantCommand(() -> {
                    logAutoEvent("shooterOn");
                    if (m_autoShooterHoldCommand != null) {
                        CommandScheduler.getInstance().cancel(m_autoShooterHoldCommand);
                    }
                    m_autoShooterHoldCommand = getAssistedShooterCommandLoop();
                    CommandScheduler.getInstance().schedule(m_autoShooterHoldCommand);
                })
            );

            // shooterOff: soltar RT (regresar a idle)
            NamedCommands.registerCommand(
                "shooterOff",
                new InstantCommand(() -> {
                    logAutoEvent("shooterOff");
                    if (m_autoShooterHoldCommand != null) {
                        CommandScheduler.getInstance().cancel(m_autoShooterHoldCommand);
                    }
                    m_autoShooterHoldCommand = null;
                    m_asistedShooter.stop();
                    m_shooter.clearAssistedRequest();
                    m_shooter.setIdleSpinEnabled(true);
                    m_shooter.endShootAndReturnToIdle();
                    m_shooter.stopBelt();
                    m_auxMotor.stop();
                })
            );

            // auxOn: bandas + auxiliar (como botón X)
            NamedCommands.registerCommand(
                "auxOn",
                new InstantCommand(() -> {
                    logAutoEvent("auxOn");
                    m_shooter.startBelt();
                    m_auxMotor.startReverseAux();
                })
            );

            // auxOff: apagar bandas y auxiliar
            NamedCommands.registerCommand(
                "auxOff",
                new InstantCommand(() -> {
                    logAutoEvent("auxOff");
                    m_shooter.stopBelt();
                    m_auxMotor.stop();
                })
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
                    new InstantCommand(() -> logAutoEvent("ShooterOn")),
                    getAutoAimToggleCommand().onlyIf(() -> !m_robotDrive.isAutoAimEnabled()),
                    Commands.waitSeconds(0.1),
                    Commands.parallel(
                        // Shooter (assisted/fallback) por 6s (1s precarga + 5s con bandas)
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
                        }, m_asistedShooter).withTimeout(6.0),
                        // Esperar 1s y luego prender bandas por 5s
                        Commands.sequence(
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
                                .withTimeout(5.0)
                        )
                    ),
                    // Al terminar el evento, apagar shooter y bandas
                    new InstantCommand(() -> {
                        m_asistedShooter.stop();
                        m_shooter.clearAssistedRequest();
                        m_shooter.stop();
                        m_shooter.stopBelt();
                        m_auxMotor.stop();
                    }, m_asistedShooter, m_shooter, m_auxMotor)
                )
            );

            // ShooterOff: apagar shooter, autoaim y bandas
            NamedCommands.registerCommand(
                "ShooterOff",
                new InstantCommand(() -> {
                    logAutoEvent("ShooterOff");
                    m_asistedShooter.stop();
                    m_shooter.clearAssistedRequest();
                    // Stop total del shooter
                    m_shooter.stop();
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
            m_autoConfigured = true;
            SmartDashboard.putBoolean("Auto/Configured", true);
            rebuildAutoChooser();
        } catch (Exception e) {
            DriverStation.reportError("PathPlanner configure failed: " + e, e.getStackTrace());
            m_autoConfigured = false;
            SmartDashboard.putBoolean("Auto/Configured", false);
            SmartDashboard.putString("Auto/ConfigureError", String.valueOf(e));
            SmartDashboard.putString("Auto/ConfigureErrorType", e.getClass().getName());

            // FALLBACK: aunque el config falle, al menos listar autos por nombre.
            tryPopulateChooserNamesOnly(String.valueOf(e));
        }
    }

    private Command build30DerechaFallback() {
        if (!m_autoConfigured) {
            return buildOpenLoopAuto();
        }

        try {
            PathPlannerPath path1 = PathPlannerPath.fromPathFile("1Path1");
            PathPlannerPath path2 = PathPlannerPath.fromPathFile("1Path2");
            return Commands.sequence(
                AutoBuilder.followPath(path1),
                AutoBuilder.followPath(path2)
            );
        } catch (Exception ex) {
            DriverStation.reportError("Fallback auto load failed: " + ex, ex.getStackTrace());
            return buildOpenLoopAuto();
        }
    }

    private Command buildOpenLoopAuto() {
        return Commands.sequence(
            Commands.run(
                    () -> m_robotDrive.drive(0.4, 0.0, 0.0, true),
                    m_robotDrive)
                .withTimeout(1.5),
            new InstantCommand(() -> m_robotDrive.drive(0.0, 0.0, 0.0, true), m_robotDrive)
        );
    }

    private void validatePPSettingsJson() {
        try {
            Path settingsPath = Filesystem.getDeployDirectory().toPath().resolve("pathplanner").resolve("settings.json");
            if (!Files.exists(settingsPath)) {
                DriverStation.reportWarning("PathPlanner settings.json missing", false);
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

            if (missing.length() > 0) {
                DriverStation.reportWarning("PathPlanner settings.json missing keys: " + missing, false);
            }
        } catch (Exception ex) {
            DriverStation.reportWarning("PathPlanner settings.json parse error: " + ex, false);
        }
    }

    private void tryPopulateChooserNamesOnly(String error) {
        try {
            Path autosDir = Filesystem.getDeployDirectory().toPath().resolve("pathplanner").resolve("autos");
            m_autoOptions.clear();

            Command doNothing = new InstantCommand();
            m_autoChooser.setDefaultOption("DoNothing", doNothing);
            m_autoOptions.put("DoNothing", doNothing);

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

            for (String base : autos) {
                // Comando placeholder: para que el chooser muestre opciones aunque la config esté mal.
                Command placeholder = new InstantCommand(() -> DriverStation.reportError(
                    "Auto '" + base + "' selected but PathPlanner is not configured. Error: " + error,
                    false));
                m_autoChooser.addOption(base, placeholder);
                m_autoOptions.put(base, placeholder);
            }

            SmartDashboard.putData("Auto/Chooser", m_autoChooser);

        } catch (Exception ex) {
            DriverStation.reportError("Auto fallback chooser failed: " + ex, ex.getStackTrace());
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

        // B (toggle): ciclo angular 5 rotaciones atras y 5 adelante, vuelve a HOME al apagar
        m_driverController.b().onTrue(
            new InstantCommand(() -> {
                m_angularOscillateActive = !m_angularOscillateActive;
                if (m_angularOscillateActive) {
                    double current = m_angular.getPositionDeg();
                    m_angularOscillateHomeDeg = current;
                    m_angularOscillateBackDeg = current - kAngularOscillateDeltaDeg;
                    m_angularOscillateForward = false;
                    CommandScheduler.getInstance().schedule(m_angularOscillateCommand);
                } else {
                    CommandScheduler.getInstance().cancel(m_angularOscillateCommand);
                    m_angular.irAPosicion(kAngularHomeDeg);
                }
            }, m_angular)
        );

        // A/Y del driver libres (DIO ahora son estados automáticos)

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
            new InstantCommand(() -> m_angular.irAPosicion(-(360.0 * 30)), m_angular)
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

        // Driver POVRight (while held): alinear al apriltag más cercano a 1m
        m_driverController.povRight().onTrue(m_alignCommand);

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
        Command selected = m_autoChooser.getSelected();
        String selectedName = "<unknown>";
        if (selected == null) {
            selectedName = "<null>";
        } else {
            for (Map.Entry<String, Command> entry : m_autoOptions.entrySet()) {
                if (entry.getValue() == selected || entry.getValue().equals(selected)) {
                    selectedName = entry.getKey();
                    break;
                }
            }
        }
        SmartDashboard.putString("Auto/Selected", selectedName);
        if (!m_autoConfigured && "30Derecha".equals(selectedName)) {
            DriverStation.reportWarning("AutoBuilder not configured; forcing open-loop auto", false);
            return buildOpenLoopAuto();
        }
        return selected;
    }

}