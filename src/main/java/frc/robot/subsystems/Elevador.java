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


public class Elevador extends SubsystemBase {
  private final SparkMax m_elevadorMotor = new SparkMax(54, MotorType.kBrushless);
  private final SparkClosedLoopController m_pidController;
  private final RelativeEncoder m_encoder;
  private double m_targetPosition = 0;
  private boolean m_abajo = false;
  private static final double POS_ABAJO_GRADOS = -(360.0 * 28);
  private static final double ABAJO_TOLERANCIA_GRADOS = (360*15.0);
  private static final double SETPOINT_TOLERANCIA_GRADOS = (360*20.0);

  public Elevador() {
    SparkMaxConfig config = new SparkMaxConfig();

    // 1. LIMITES
    config.smartCurrentLimit(40); 
    config.voltageCompensation(12.0);

    // 2. ELIMINAMOS MAX MOTION (Causaba el arranque lento)
    // No configuramos nada de maxVelocity ni maxAcceleration aquí.
    
    // 3. PID SIMPLE (Calculado para NO vibrar)
    config.closedLoop
        // P: 0.00015
        // Matemáticas: Error de 8000 grados * 0.00015 = 1.2 (100% Salida)
        // Esto le dará fuerza total al inicio para arrancar, y bajará al llegar.
        .p(0.00015, ClosedLoopSlot.kSlot0) 
        
        .i(0.0, ClosedLoopSlot.kSlot0)
        .d(0.0, ClosedLoopSlot.kSlot0) // Sin D para evitar ruido
        .velocityFF(0.0, ClosedLoopSlot.kSlot0) // Sin FF en modo Position
        .outputRange(-0.5, 0.5, ClosedLoopSlot.kSlot0); // Limitado al 50% por seguridad

    // 4. ENCODER
    config.encoder.positionConversionFactor(360.0);

  m_elevadorMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  m_pidController = m_elevadorMotor.getClosedLoopController();
  m_encoder = m_elevadorMotor.getEncoder();
    
  m_encoder.setPosition(0);
    m_targetPosition = 0;
  }

  public void irAPosicion(double grados) {
    m_targetPosition = grados;
    System.out.println("Yendo a (DIRECTO): " + grados);
    
  // CAMBIO: usamos kPosition en vez de kMAXMotionPositionControl
  m_pidController.setReference(
    m_targetPosition, 
  ControlType.kPosition,  // posicion directa
    ClosedLoopSlot.kSlot0
  );
  }

  public double getPosicion() {
    return m_encoder.getPosition();
  }
  
  public void resetEncoder() {
      m_encoder.setPosition(0);
      m_targetPosition = 0;
      System.out.println("Encoder Reset");
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
    SmartDashboard.putNumber("Elevador/Pos", m_encoder.getPosition());
    SmartDashboard.putNumber("Elevador/Target", m_targetPosition);
    SmartDashboard.putNumber("Elevador/Output", m_elevadorMotor.getAppliedOutput());
    // Error actual para debug
    SmartDashboard.putNumber("Elevador/Error", m_targetPosition - m_encoder.getPosition());
    SmartDashboard.putBoolean("Elevador/Abajo", m_abajo);
  }
}

