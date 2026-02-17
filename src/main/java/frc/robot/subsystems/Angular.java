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

  // --- Anti-atoradas cycle (toggle) ---
  private boolean m_unjamEnabled = false;
  private UnjamState m_unjamState = UnjamState.IDLE;
  private double m_unjamNextActionTimeSec = 0.0;

  private static final double POS_ARRIBA_RELPOS_GRADOS = (360.0 * 10.0); // 10 rotaciones arriba
  private static final double UNJAM_WAIT_UP_SEC = 0.2;
  private static final double UNJAM_WAIT_DOWN_SEC = 0.2;

  // Objetivos del ciclo (se setean al iniciar para que NO se acumulen offsets)
  private double m_unjamUpTargetDeg = 0.0;

  private enum UnjamState {
    IDLE,
    GOING_UP,
    WAITING_AT_TOP,
    GOING_DOWN,
    WAITING_AT_BOTTOM
  }

  private static final double POS_ABAJO_GRADOS = -(360.0 * 28);
  private static final double ABAJO_TOLERANCIA_GRADOS = (360 * 15.0);
  private static final double SETPOINT_TOLERANCIA_GRADOS = (360 * 20.0);

  // --- Manual open-loop mode (para calibración) ---
  private boolean m_manualOpenLoop = false;

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

  public double getPosicion() {
    return m_encoder.getPosition();
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

  public boolean isAbajo() {
    return m_abajo;
  }

  /**
   * Toggle del ciclo anti-atoradas.
   * Solo se puede INICIAR si ya está abajo (m_abajo == true).
   * Si se vuelve a llamar mientras está activo, se detiene y deja el motor yendo a la posición abajo.
   */
  public void toggleUnjamCycle() {
    if (m_unjamEnabled) {
      m_unjamEnabled = false;
      m_unjamState = UnjamState.IDLE;
      // Al parar, regresamos a abajo para quedar seguro.
      irAPosicion(POS_ABAJO_GRADOS);
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
    m_unjamUpTargetDeg = POS_ABAJO_GRADOS + POS_ARRIBA_RELPOS_GRADOS;
    irAPosicion(m_unjamUpTargetDeg);
  }

  public boolean isUnjamEnabled() {
    return m_unjamEnabled;
  }

  public boolean atSetpoint() {
    return Math.abs(m_targetPosition - m_encoder.getPosition()) <= SETPOINT_TOLERANCIA_GRADOS;
  }

  @Override
  public void periodic() {
    m_abajo = Math.abs(m_encoder.getPosition() - POS_ABAJO_GRADOS) <= ABAJO_TOLERANCIA_GRADOS;

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
          if (Math.abs(m_unjamUpTargetDeg - m_encoder.getPosition()) <= SETPOINT_TOLERANCIA_GRADOS) {
            m_unjamState = UnjamState.WAITING_AT_TOP;
            m_unjamNextActionTimeSec = now + UNJAM_WAIT_UP_SEC;
          }
          break;

        case WAITING_AT_TOP:
          if (now >= m_unjamNextActionTimeSec) {
            m_unjamState = UnjamState.GOING_DOWN;
            irAPosicion(POS_ABAJO_GRADOS);
          }
          break;

        case GOING_DOWN:
          // Al llegar abajo, esperamos un rato antes de volver a subir.
          if (m_abajo) {
            m_unjamState = UnjamState.WAITING_AT_BOTTOM;
            m_unjamNextActionTimeSec = now + UNJAM_WAIT_DOWN_SEC;
          }
          break;

        case WAITING_AT_BOTTOM:
          if (now >= m_unjamNextActionTimeSec) {
            m_unjamState = UnjamState.GOING_UP;
            irAPosicion(m_unjamUpTargetDeg);
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
  }
}

