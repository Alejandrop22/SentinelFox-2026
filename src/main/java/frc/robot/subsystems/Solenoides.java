package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Solenoides extends SubsystemBase {
    private final PneumaticHub pneumaticHub;
    private final Solenoid extendA;
    private final Solenoid extendB;
    private final Solenoid retractA;
    private final Solenoid retractB;
    private boolean m_extended = false;

    public Solenoides() {
        this(60, 0, 1, 2, 3);
    }

    public Solenoides(int canIdHub, int aExtendChannel, int bExtendChannel, int aRetractChannel, int bRetractChannel) {
        pneumaticHub = new PneumaticHub(canIdHub);
        pneumaticHub.enableCompressorDigital();

        extendA = pneumaticHub.makeSolenoid(aExtendChannel);
        extendB = pneumaticHub.makeSolenoid(bExtendChannel);
        retractA = pneumaticHub.makeSolenoid(aRetractChannel);
        retractB = pneumaticHub.makeSolenoid(bRetractChannel);
        apagarAmbos();
    }

    public void extenderAmbos() {
        extendA.set(true);
        extendB.set(true);
        retractA.set(false);
        retractB.set(false);
        m_extended = true;
    }

    public void retraerAmbos() {
        extendA.set(false);
        extendB.set(false);
        retractA.set(true);
        retractB.set(true);
        m_extended = false;
    }

    public void apagarAmbos() {
        extendA.set(false);
        extendB.set(false);
        retractA.set(false);
        retractB.set(false);
    }

    public void alternarAmbos() {
        if (m_extended) {
            retraerAmbos();
            return;
        }
        extenderAmbos();
    }
}
