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

/**
 * Subsystem para el motor "Angular" del intake.
 *
 * Nota: Este subsystem es el motor "Angular" del intake (CAN 54).
 */
public class Angular extends SubsystemBase {
  private final SparkMax m_angularMotor = new SparkMax(54, MotorType.kBrushless);
  private final SparkClosedLoopController m_pidController;
  private final RelativeEncoder m_encoder;

  private double m_targetPosition = 0;
  private boolean m_abajo = false;

  private static final double POS_ABAJO_GRADOS = -(360.0 * 28);
  private static final double ABAJO_TOLERANCIA_GRADOS = (360 * 15.0);
  private static final double SETPOINT_TOLERANCIA_GRADOS = (360 * 20.0);

  public Angular() {
    SparkMaxConfig config = new SparkMaxConfig();

    // 1. LIMITES
    config.smartCurrentLimit(40);
    config.voltageCompensation(12.0);

    // 2. PID SIMPLE (Calculado para NO vibrar)
    config.closedLoop
        // P: 0.00015
        // Matemáticas: Error de 8000 grados * 0.00015 = 1.2 (100% Salida)
        .p(0.00015, ClosedLoopSlot.kSlot0)
        .i(0.0, ClosedLoopSlot.kSlot0)
        .d(0.0, ClosedLoopSlot.kSlot0)
        .outputRange(-0.5, 0.5, ClosedLoopSlot.kSlot0);

    // 3. ENCODER
    config.encoder.positionConversionFactor(360.0);

  // Usamos los mismos flags que el resto del proyecto; WPILib/REV pueden marcarlos deprecated
  // pero siguen funcionando en 2026.
  m_angularMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    m_pidController = m_angularMotor.getClosedLoopController();
    m_encoder = m_angularMotor.getEncoder();

    m_encoder.setPosition(0);
    m_targetPosition = 0;
  }

  public void irAPosicion(double grados) {
    m_targetPosition = grados;
    System.out.println("Angular: yendo a (DIRECTO): " + grados);

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
    System.out.println("Angular: Encoder Reset");
  }

  public boolean isAbajo() {
    return m_abajo;
  }

  public boolean atSetpoint() {
    return Math.abs(m_targetPosition - m_encoder.getPosition()) <= SETPOINT_TOLERANCIA_GRADOS;
  }

  @Override
  public void periodic() {
    m_abajo = Math.abs(m_encoder.getPosition() - POS_ABAJO_GRADOS) <= ABAJO_TOLERANCIA_GRADOS;

    SmartDashboard.putNumber("Angular/Pos", m_encoder.getPosition());
    SmartDashboard.putNumber("Angular/Target", m_targetPosition);
    SmartDashboard.putNumber("Angular/Output", m_angularMotor.getAppliedOutput());
    SmartDashboard.putNumber("Angular/Error", m_targetPosition - m_encoder.getPosition());
    SmartDashboard.putBoolean("Angular/Abajo", m_abajo);
  }
}

