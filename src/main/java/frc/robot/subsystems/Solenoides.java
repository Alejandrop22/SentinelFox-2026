package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Solenoides extends SubsystemBase {
    private final PneumaticHub pneumaticHub;
    private final DoubleSolenoid solenoideA;
    private final DoubleSolenoid solenoideB;

    public Solenoides() {
        this(1, 0, 1, 2, 3);
    }

    public Solenoides(int canIdHub, int aForwardChannel, int aReverseChannel, int bForwardChannel, int bReverseChannel) {
        pneumaticHub = new PneumaticHub(canIdHub);
        pneumaticHub.enableCompressorDigital();

        solenoideA = pneumaticHub.makeDoubleSolenoid(aForwardChannel, aReverseChannel);
        solenoideB = pneumaticHub.makeDoubleSolenoid(bForwardChannel, bReverseChannel);
    }

    public void extenderAmbos() {
        solenoideA.set(DoubleSolenoid.Value.kForward);
        solenoideB.set(DoubleSolenoid.Value.kForward);
    }

    public void retraerAmbos() {
        solenoideA.set(DoubleSolenoid.Value.kReverse);
        solenoideB.set(DoubleSolenoid.Value.kReverse);
    }

    public void apagarAmbos() {
        solenoideA.set(DoubleSolenoid.Value.kOff);
        solenoideB.set(DoubleSolenoid.Value.kOff);
    }

    public void alternarAmbos() {
        if (solenoideA.get() == DoubleSolenoid.Value.kForward) {
            retraerAmbos();
        } else {
            extenderAmbos();
        }
    }
}
