package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

/**
 * Simple one-motor subsystem for the motor at CAN ID 52.
 * Provides a toggleable reverse-50% output and stop.
 */
public class AuxMotor extends SubsystemBase {
  private final SparkMax m_motor52 = new SparkMax(52, MotorType.kBrushless);

  /** Run the motor at -50% (reverse). */
  public void startReverse50() {
    m_motor52.set(-0.5);
  }

  /** Stop the motor. */
  public void stop() {
    m_motor52.set(0.0);
  }
}
