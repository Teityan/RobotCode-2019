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

    private boolean is_RollerMoving;

    /*
    Grabber(Motor motor, Encoder encoder, Solenoid solenoid)
        モーターとエンコーダー、ソレノイドを受け取る

    holdCargo():
        モーターを回してCARGOを回収する
    releaseCargo();
        モーターを回してCARGOを射出する
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
