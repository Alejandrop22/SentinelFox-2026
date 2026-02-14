package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Minimal ExampleSubsystem to satisfy template commands. */
public class ExampleSubsystem extends SubsystemBase {
  public ExampleSubsystem() {}

  /**
   * Example method that returns a simple command (no-op) that requires this subsystem.
   */
  public Command exampleMethodCommand() {
    return new InstantCommand(() -> {}, this);
  }
}
