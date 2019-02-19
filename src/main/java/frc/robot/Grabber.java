/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.VictorSP;

public class Grabber {

    private VictorSP rollerMotor;
    private Solenoid barSolenoid;
    private Solenoid armSolenoid;

    private boolean is_RollerMoving = false;


    Grabber(VictorSP rollerMotor, Solenoid barSolenoid, Solenoid armSolenoid) {
        this.rollerMotor = rollerMotor;
        this.barSolenoid = barSolenoid;
        this.armSolenoid = armSolenoid;
    }
    
	public void applyState(State state) {
        switch (state.cargoState) {
            case kHold:
                holdCargo();
                break;
            
            case kRelease:
                releaseCargo();
                break;

            case kDoNothing:
                stopRoller();
                break;
            
            default:
                break;
        }

        if (state.is_toHoldPanel) {
            // パネルをつかむ
           holdPanel();    
        } else {
            // パネルを離す
           releasePanel();    
        }

        
        if (state.is_toRetractArm) {
            // アームをしまう
            retractArm();   
        } else if(state.is_autoClimbOn) {
            // 自動クライム中はアームをしまう
            retractArm();
        } else {
            // アームを出す
            releaseArm();    
        }
    }
    
    public void holdCargo() {
        rollerMotor.set(1.0);   
        is_RollerMoving = true;
    }

    public void releaseCargo() {
        rollerMotor.set(-1.0);    
        is_RollerMoving = true;
    }

    public void stopRoller() {
        rollerMotor.stopMotor();
        is_RollerMoving = false;
    }

    public void setRollerSpeed(double speed) {
        rollerMotor.set(speed);
        is_RollerMoving = true;
    }

    public boolean is_RollerMoving() {
        return is_RollerMoving;
    }

    public void holdPanel() {
        barSolenoid.set(false);   
    }

    public void releasePanel() {
        barSolenoid.set(true);   
    }

    public void retractArm() {
        armSolenoid.set(false);   
    }

    public void releaseArm() {
        armSolenoid.set(true);    
    }


    /*
    Grabber(Motor motor, Solenoid barSolenoid, Solenoid armSolenoid)
        モーターとソレノイドを受け取る

    applyState()
        Stateから情報を受け取り入力する

    holdCargo():
        モーターを回してCARGOを回収する
    releaseCargo();
        モーターを回してCARGOを射出する
    stopRoller();
        モーターを止める
    isRollerMoving()
        モーターが動いているか

    holdPanel();
        棒を広げてHATCH PANEL回収する
    releasePanl();
        棒を狭めてロボットが後退すればHATCH PANELを置けるようにする
    

    retractArm();
        アームをしまう
    releaseArm();
        アームを出す
        
    */

}
