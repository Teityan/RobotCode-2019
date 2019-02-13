package frc.robot;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Talon;

public class Climb {

    private Talon climbMotor;
    private Solenoid climbSolenoid;

    Climb(Talon climbMotor, Solenoid climbSolenoid){
        this.climbMotor = climbMotor;
        this.climbSolenoid = climbSolenoid;
    }

    public void climbAdvance(double speed){
        climbMotor.setSpeed(speed);
    }

    public void climbLockStopper(){
        climbSolenoid.set(true);      
    }

    public void climbUnlockStopper(){
        climbSolenoid.set(false);
    }

    public void applyState(State state) {
        if (state.is_autoClimbOn) {
            switch (state.climbSequence) {
            case kDoNothing:
                break;
            case kLiftUp:
                break;
            case kLocking:
                break;
            case kLiftDown:
                break;
            }
        }
    }

}