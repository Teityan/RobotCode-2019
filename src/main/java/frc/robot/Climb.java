package frc.robot;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Talon;

public class Climb {

    private Talon climbMotor;
    private Solenoid frontClimbSolenoid;
    private Solenoid backClimbSolenoid;

    Climb(Talon climbMotor, Solenoid frontClimbSolenoid, Solenoid backClimbSolenoid){
        this.climbMotor = climbMotor;
        this.frontClimbSolenoid = frontClimbSolenoid;
        this.backClimbSolenoid = backClimbSolenoid;
    }

    public void climbAvance(double speed){
        climbMotor.setSpeed(speed);
    }

    public void climbStopperSet(boolean on){
        frontClimbSolenoid.set(on);
        backClimbSolenoid.set(on);
    }

}