package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class AuxMotor extends SubsystemBase {
  private static final int kAuxMotorCanId = 52;
  private static final double kReverse50Percent = -0.5;

  private final SparkMax m_motor52 = new SparkMax(kAuxMotorCanId, MotorType.kBrushless);

  public void startReverseAux() {
    m_motor52.set(kReverse50Percent);
  }

  public void stop() {
    m_motor52.set(0.0);
  }
}
