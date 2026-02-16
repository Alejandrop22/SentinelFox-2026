package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class AuxMotor extends SubsystemBase {
  private final SparkMax m_motor52 = new SparkMax(52, MotorType.kBrushless);

  public void startReverse50() {
    m_motor52.set(-0.5);
  }

  public void stop() {
    m_motor52.set(0.0);
  }
}
