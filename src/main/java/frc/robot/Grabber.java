/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.PWMSpeedController;
import edu.wpi.first.wpilibj.Solenoid;

/**
 * Add your docs here.
 */
public class Grabber {

    private PWMSpeedController rollerMotor;
    private Solenoid barSolenoid;
    private Solenoid armSolenoid;

    private boolean is_RollerMoving = false;

    Grabber(PWMSpeedController rollerMotor, Solenoid barSolenoid, Solenoid armSolenoid){
        this.rollerMotor = rollerMotor;
        this.barSolenoid = barSolenoid;
        this.armSolenoid = armSolenoid;
    }

	/**
	 * ToDo
	 */
	public void applyState(State state) {

        switch(state.cargoState){
            case kHold:
                releaseArm();
                holdCargo();
                break;
            
            case kRelease:
                releaseCargo();
                break;

            case kDoNothing:
                stopRoller();
                releaseArm();
                break;
            
            default:
                break;
        }

        if(state.is_toHoldPanel){
           holdPanel();    // パネルをつかむ
        }else{
           releasePanel();    // パネルを離す
        }

        if(state.is_toHoldArm){
            holdArm();
        }else{
            releaseArm();
        }
    }
    
    public void holdCargo(){
        rollerMotor.set(0.4);    //1は強すぎるので0.4に抑える
        is_RollerMoving = true;
    }

    public void releaseCargo(){
        rollerMotor.set(-0.4);    //1は強すぎるので0.4に抑える
        is_RollerMoving = true;
    }

    public void stopRoller(){
        rollerMotor.stopMotor();
        is_RollerMoving = false;
    }

    public void setRollerSpeed(double speed){
        rollerMotor.set(speed);
        is_RollerMoving = true;
    }

    public boolean is_RollerMoving(){
        return is_RollerMoving;
    }

    public void holdPanel(){
        barSolenoid.set(true);    //ソレノイドのつけ方によりT/Fは変わる
    }

    public void releasePanel(){
        barSolenoid.set(false);    //ソレノイドのつけ方によりT/Fは変わる
    }

    public void holdArm(){
        armSolenoid.set(false);   
    }

    public void releaseArm(){
        armSolenoid.set(true);    
    }


    /*
    Grabber(Motor motor, Solenoid barSolenoid, Solenoid armSolenoid)
        モーターとソレノイドを受け取る

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
    

    holdArm();
        アームをしまう
    releaseArm();
        アームを出す
        
    */

}
