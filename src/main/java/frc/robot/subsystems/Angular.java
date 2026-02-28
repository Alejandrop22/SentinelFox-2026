package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;
public class Angular extends SubsystemBase {
  private final SparkMax m_angularMotor = new SparkMax(54, MotorType.kBrushless);
  private final SparkClosedLoopController m_pidController;
  private final RelativeEncoder m_encoder;

  private double m_targetPosition = 0;
  private boolean m_abajo = false;

  // --- Anti-atoradas (one-shot) ---
  private boolean m_unjamEnabled = false;
  private UnjamState m_unjamState = UnjamState.IDLE;
  private double m_unjamNextActionTimeSec = 0.0;

  private static final double kUnjamUpRelativePositionDeg = (360.0 * 10.0); // 10 rotaciones arriba
  private static final double kUnjamWaitUpSec = 0.2;
  private static final double kUnjamWaitDownSec = 0.2;

  // Objetivos del ciclo (se setean al iniciar para que NO se acumulen offsets)
  private double m_unjamUpTargetDeg = 0.0;

  private enum UnjamState {
    IDLE,
    GOING_UP,
    WAITING_AT_TOP,
    GOING_DOWN,
    WAITING_AT_BOTTOM
  }

  private static final double kDownPositionDeg = -(360.0 * 28);
  private static final double kDownToleranceDeg = (360 * 15.0);
  private static final double kSetpointToleranceDeg = (360 * 20.0);

  // --- Manual open-loop mode (para calibración) ---
  private boolean m_manualOpenLoop = false;

  // --- Auto-bajar hasta stall y subir 2 rotaciones ---
  private enum AutoDownState {
    IDLE,
    GOING_DOWN_OPENLOOP,
    BACKING_OFF
  }

  private AutoDownState m_autoDownState = AutoDownState.IDLE;
  private double m_autoDownStartTimeSec = 0.0;
  private double m_autoDownStallStartTimeSec = 0.0;
  private double m_autoDownBackoffTargetDeg = 0.0;

  // Tunables (ajusta en práctica)
  private static final double kAutoDownPercent = -0.25; // % negativo para bajar
  private static final double kAutoDownTimeoutSec = 2.0;
  private static final double kStallCurrentA = 35.0;
  private static final double kStallVelocityDegPerSec = 50.0;
  private static final double kStallDebounceSec = 0.12;
  private static final double kAutoBackoffRotations = 3.0; // subir N rotaciones (después de detectar stall)

  public Angular() {
    SparkMaxConfig config = new SparkMaxConfig();
    config.smartCurrentLimit(40);
    config.voltageCompensation(12.0);
    config.closedLoop
        .p(0.00015, ClosedLoopSlot.kSlot0)
        .i(0.0, ClosedLoopSlot.kSlot0)
        .d(0.0, ClosedLoopSlot.kSlot0)
        .outputRange(-0.5, 0.5, ClosedLoopSlot.kSlot0);
    config.encoder.positionConversionFactor(360.0);
  m_angularMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    m_pidController = m_angularMotor.getClosedLoopController();
    m_encoder = m_angularMotor.getEncoder();

    m_encoder.setPosition(0);
    m_targetPosition = 0;
  }

  public void irAPosicion(double grados) {
    m_manualOpenLoop = false;
    m_targetPosition = grados;
    m_pidController.setReference(
        m_targetPosition,
        ControlType.kPosition,
        ClosedLoopSlot.kSlot0);
  }

  public void resetEncoder() {
    m_encoder.setPosition(0);
    m_targetPosition = 0;
  }

  /**
   * Manejo directo del motor (sin PID). Útil para calibración/manual.
   * También apaga el ciclo anti-atoradas para que no "pelee".
   */
  public void setOpenLoopPercent(double percent) {
    m_unjamEnabled = false;
    m_unjamState = UnjamState.IDLE;
    m_manualOpenLoop = true;
    m_angularMotor.set(percent);
  }

  public void stopOpenLoop() {
    if (m_manualOpenLoop) {
      m_angularMotor.set(0.0);
    }
    m_manualOpenLoop = false;
  }

  /**
   * Baja el Angular en open-loop (negativo) hasta detectar stall y luego sube 2 rotaciones.
   * Es no-bloqueante: corre en periodic() con una máquina de estados.
   */
  public void startAutoDownToStallAndBackoff() {
    // Cancelar cualquier control manual y el unjam para que no peleen.
    m_unjamEnabled = false;
    m_unjamState = UnjamState.IDLE;
    m_manualOpenLoop = false;

    m_autoDownState = AutoDownState.GOING_DOWN_OPENLOOP;
    m_autoDownStartTimeSec = Timer.getFPGATimestamp();
    m_autoDownStallStartTimeSec = 0.0;
    m_angularMotor.set(kAutoDownPercent);
  }


  private void stopUnjamNow() {
    m_unjamEnabled = false;
    m_unjamState = UnjamState.IDLE;
    // Al parar, regresamos a abajo para quedar seguro.
    irAPosicion(kDownPositionDeg);
  }

  /**
   * Inicia 1 ciclo anti-atoradas.
   * Requiere que el mecanismo ya esté abajo (m_abajo == true).
   * Si se llama mientras está activo, lo cancela y regresa a abajo.
   */
  private void startUnjamOneShot() {
    if (m_unjamEnabled) {
      stopUnjamNow();
      return;
    }

    // Solo arrancar si ya está abajo
    if (!m_abajo) {
      return;
    }

    m_unjamEnabled = true;
    m_unjamState = UnjamState.GOING_UP;
    // Calculamos el objetivo 'arriba' una sola vez para evitar drift/acumulación.
    // Asumimos que al iniciar estamos abajo.
    m_unjamUpTargetDeg = kDownPositionDeg + kUnjamUpRelativePositionDeg;
    irAPosicion(m_unjamUpTargetDeg);
  }

  private void stopUnjamAfterCycle() {
    // Termina el ciclo y deja el mecanismo yendo/estando abajo.
    m_unjamEnabled = false;
    m_unjamState = UnjamState.IDLE;
    irAPosicion(kDownPositionDeg);
  }

  /**
   * Dispara 1 ciclo anti-atoradas si el mecanismo ya está abajo.
   * (Se deja el nombre para no romper llamadas existentes.)
   */
  public void setUnjamCycleEnabled(boolean enabled) {
    if (!enabled) {
      // Binding actual es one-shot; ignoramos 'false' para que no cancele al soltar.
      return;
    }

    startUnjamOneShot();
  }

  @Override
  public void periodic() {
    m_abajo = Math.abs(m_encoder.getPosition() - kDownPositionDeg) <= kDownToleranceDeg;

    // --- Auto down-to-stall routine ---
    if (m_autoDownState != AutoDownState.IDLE) {
      double now = Timer.getFPGATimestamp();
      double currentA = m_angularMotor.getOutputCurrent();
      double velDegPerSec = m_encoder.getVelocity(); // con conversionFactor=360, velocity queda en deg/s aprox

      SmartDashboard.putString("Angular/AutoDownState", m_autoDownState.toString());
      SmartDashboard.putNumber("Angular/AutoDown_CurrentA", currentA);
      SmartDashboard.putNumber("Angular/AutoDown_VelDegPerSec", velDegPerSec);

      switch (m_autoDownState) {
        case GOING_DOWN_OPENLOOP:
          // Timeout de seguridad
          if (now - m_autoDownStartTimeSec > kAutoDownTimeoutSec) {
            m_angularMotor.set(0.0);
            m_autoDownState = AutoDownState.IDLE;
            break;
          }

          boolean stallNow = (Math.abs(velDegPerSec) < kStallVelocityDegPerSec) && (currentA >= kStallCurrentA);
          if (stallNow) {
            if (m_autoDownStallStartTimeSec <= 0.0) {
              m_autoDownStallStartTimeSec = now;
            }
            if (now - m_autoDownStallStartTimeSec >= kStallDebounceSec) {
              // Stall confirmado -> parar y hacer backoff
              m_angularMotor.set(0.0);
              m_autoDownBackoffTargetDeg = m_encoder.getPosition() + (kAutoBackoffRotations * 360.0);
              irAPosicion(m_autoDownBackoffTargetDeg);
              m_autoDownState = AutoDownState.BACKING_OFF;
            }
          } else {
            m_autoDownStallStartTimeSec = 0.0;
            // seguir bajando
            m_angularMotor.set(kAutoDownPercent);
          }
          break;

        case BACKING_OFF:
          if (Math.abs(m_autoDownBackoffTargetDeg - m_encoder.getPosition()) <= kSetpointToleranceDeg) {
            m_autoDownState = AutoDownState.IDLE;
          }
          break;

        case IDLE:
        default:
          m_autoDownState = AutoDownState.IDLE;
          break;
      }

      // En autoDown no corremos unjam ni manual.
      SmartDashboard.putNumber("Angular/Pos", m_encoder.getPosition());
      SmartDashboard.putNumber("Angular/Target", m_targetPosition);
      SmartDashboard.putNumber("Angular/Output", m_angularMotor.getAppliedOutput());
      SmartDashboard.putNumber("Angular/Error", m_targetPosition - m_encoder.getPosition());
      SmartDashboard.putBoolean("Angular/Abajo", m_abajo);
      SmartDashboard.putBoolean("Angular/UnjamEnabled", m_unjamEnabled);
      SmartDashboard.putBoolean("Angular/ManualOpenLoop", m_manualOpenLoop);
      SmartDashboard.putBoolean("Angular/AutoDownActive", true);
      return;
    }

    // Si estamos en modo manual, no ejecutar la máquina de estados del unjam.
    if (m_manualOpenLoop) {
      SmartDashboard.putNumber("Angular/Pos", m_encoder.getPosition());
      SmartDashboard.putNumber("Angular/Target", m_targetPosition);
      SmartDashboard.putNumber("Angular/Output", m_angularMotor.getAppliedOutput());
      SmartDashboard.putNumber("Angular/Error", m_targetPosition - m_encoder.getPosition());
      SmartDashboard.putBoolean("Angular/Abajo", m_abajo);
      SmartDashboard.putBoolean("Angular/UnjamEnabled", m_unjamEnabled);
      SmartDashboard.putBoolean("Angular/ManualOpenLoop", m_manualOpenLoop);
      return;
    }

    // Máquina de estados del ciclo anti-atoradas (sin sleeps)
    if (m_unjamEnabled) {
      double now = Timer.getFPGATimestamp();

      switch (m_unjamState) {
        case GOING_UP:
          if (Math.abs(m_unjamUpTargetDeg - m_encoder.getPosition()) <= kSetpointToleranceDeg) {
            m_unjamState = UnjamState.WAITING_AT_TOP;
            m_unjamNextActionTimeSec = now + kUnjamWaitUpSec;
          }
          break;

        case WAITING_AT_TOP:
          if (now >= m_unjamNextActionTimeSec) {
            m_unjamState = UnjamState.GOING_DOWN;
            irAPosicion(kDownPositionDeg);
          }
          break;

        case GOING_DOWN:
          // Al llegar abajo, esperamos un rato antes de volver a subir.
          if (m_abajo) {
            m_unjamState = UnjamState.WAITING_AT_BOTTOM;
            m_unjamNextActionTimeSec = now + kUnjamWaitDownSec;
          }
          break;

        case WAITING_AT_BOTTOM:
          if (now >= m_unjamNextActionTimeSec) {
            // One-shot: al completar la bajada + espera, terminamos el ciclo.
            stopUnjamAfterCycle();
          }
          break;

        case IDLE:
        default:
          break;
      }
    }

    SmartDashboard.putNumber("Angular/Pos", m_encoder.getPosition());
    SmartDashboard.putNumber("Angular/Target", m_targetPosition);
    SmartDashboard.putNumber("Angular/Output", m_angularMotor.getAppliedOutput());
    SmartDashboard.putNumber("Angular/Error", m_targetPosition - m_encoder.getPosition());
    SmartDashboard.putBoolean("Angular/Abajo", m_abajo);
    SmartDashboard.putBoolean("Angular/UnjamEnabled", m_unjamEnabled);
    SmartDashboard.putBoolean("Angular/ManualOpenLoop", m_manualOpenLoop);
    SmartDashboard.putBoolean("Angular/AutoDownActive", false);
  }
}

