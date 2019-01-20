/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Encoder;
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


    Grabber(PWMSpeedController rollerMotor, Solenoid barSolenoid, Solenoid armSolenoid, Encoder liftEncoder){
        this.rollerMotor = rollerMotor;
        this.barSolenoid = barSolenoid;
        this.armSolenoid = armSolenoid;
    }

    public void holdCargo(){
        rollerMotor.set(1.0);//設置によっては±変わる
        is_RollerMoving = true;
    }

    public void releaseCargo(){
        rollerMotor.set(-1.0);//設置によっては変わる
        is_RollerMoving = true;
    }

    public void stopRoller(){
        rollerMotor.stopMotor();
        is_RollerMoving = false;
    }

    public boolean isRollerMoving(){
        return is_RollerMoving;
    }

    public void holdPanel(){
        barSolenoid.set(true);//ソレノイドのつけ方によりT/Fは変わる
    }

    public void releasePanel(){
        barSolenoid.set(false);//ソレノイドのつけ方によりT/Fは変わる
    }

    public void holdArm(){
        armSolenoid.set(true);//ソレノイドのつけ方によりT/Fは変わる
    }

    public void releaseArm(){
        armSolenoid.set(false);//ソレノイドのつけ方によりT/Fは変わる
    }

    /*
    Grabber(Motor motor, Solenoid barSolenoid, Solenoid armSolenoid, Encoder encoder)
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
        モーターや棒、板をしまう
    releaseArm();
        モーターや棒、板を出す
    */

}
