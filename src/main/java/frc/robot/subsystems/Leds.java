package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DriverStation;

public class Leds {

    //Solo jala el state 2 en adelante, el 1 es para reiniciar el wled
    private final DigitalOutput state_1 = new DigitalOutput(0);
    private final DigitalOutput state_2 = new DigitalOutput(1);
    private final DigitalOutput state_3 = new DigitalOutput(2);
    private final DigitalOutput state_4 = new DigitalOutput(3);
    private final DigitalOutput state_5 = new DigitalOutput(4);
    
    private boolean isIntaking = false;
    private boolean isAuxRunning = false;
    private boolean isAutoAiming = false;
    private boolean isAutoShooting = false;

     public void setIntaking(boolean intaking) {
            isIntaking = intaking;
            isAuxRunning = false; // Asegura que no se muestre el estado de aux si se está intaking
            isAutoAiming = false; // Asegura que no se muestre el estado de auto aiming si se está intaking
            isAutoShooting = false; // Asegura que no se muestre el estado de auto shooting si se está intaking
            ledUpdate();
        }
    
    public void setAuxRunning(boolean auxRunning) {
            isAuxRunning = auxRunning;
            isIntaking = false; // Asegura que no se muestre el estado de intaking si se está corriendo el aux
            isAutoAiming = false; // Asegura que no se muestre el estado
            isAutoShooting = false; // Asegura que no se muestre el estado de auto shooting si se está corriendo el aux
            ledUpdate();
        }

    public void setAutoAiming(boolean autoAiming) {
            isAutoAiming = autoAiming;
            isIntaking = false; // Asegura que no se muestre el estado de intaking si se está auto aiming
            isAuxRunning = false; // Asegura que no se muestre el estado de aux si se está auto aiming
            isAutoShooting = false; // Asegura que no se muestre el estado de auto shooting si se está auto aiming
            ledUpdate();
        }

    public void setAutoShooting(boolean autoShooting) {
            isAutoShooting = autoShooting;
            isIntaking = false; // Asegura que no se muestre el estado de intaking si se está auto shooting
            isAuxRunning = false; // Asegura que no se muestre el estado de aux si se está auto shooting
            isAutoAiming = false; // Asegura que no se muestre el estado de auto aiming si se está auto shooting
            ledUpdate();
        }

    public void ledUpdate() {

        if(DriverStation.isEnabled()) {

            //Si no hay nada prendido pon el default, el state 2.

            if(isIntaking || isAuxRunning || isAutoAiming || isAutoShooting) {
                state_2.set(false);
            } else {
                state_2.set(true);
            }

            state_3.set(isIntaking);
            state_4.set(isAuxRunning);
            state_5.set(isAutoAiming || isAutoShooting);

        } else {

            // si el robot esta apagado apaga todo alv
            
            state_1.set(false);
            state_2.set(false);
            state_3.set(false);
            state_4.set(false);
            state_5.set(false);   
        }


    }










    
}
