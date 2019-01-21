/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.          
                                                     */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PWMSpeedController;
import edu.wpi.first.wpilibj.SendableBase;
public class Lift {
    /*
    <モーター>
    モーターの速度を変更する
    set(数字);
    数字:-1.0~+1.0
    <エンコーダー>
    getSpeed()
        速度を得る
    getHeight()    
        位置を得る
    cmで

    <PID>
    enablePID()
        PIDを有効にする
    disablePID()
        PIDを無効にする
    setSetpoint(double height)
    setSetpoint(LiftPosition position)
        grubberの高さを指定する
        disableだったら動かない
        セットポイントの数:
            カーゴシップのハッチ付ける位置
            カーゴシップのボール入れる位置
            ロケットのハッチ付ける位置
                1~2段目
            ロケットのボール入れる位置
                1~2段目
    */
    Encoder encoder;
    PWMSpeedController P_SpeedController;
    PIDController controller;
    //コンストラクター
    Lift(PWMSpeedController P_SpeedController, Encoder encoder, PIDController controller){
        this.encoder = encoder;
        this.P_SpeedController = P_SpeedController;
        this.controller = controller;
    }

    //モーター
    public void setSpeed(double speed){
        P_SpeedController.set(speed);
    }
    //エンコーダー
    public double getHeight(){
        double height = encoder.getDistance();
        return height;
}
    public double getSpeed(){
        double speed = encoder.getRate();
        return speed;
    }

    //PID
    public void enablePID(){
        controller.enable();
    }
    public void disablePID(){
        controller.disable();
    }
    public void setSetpoint(double height){
        controller.setSetpoint(height);
    }
    public void setSetopoint(point point){
        if(controller.isEnabled()) disablePID();
        enablePID();
        switch(point){
            case cagoHach:
            controller.setSetpoint(1.5);
            break;
            case cagoBall:
            controller.setSetpoint(1.4);
            break;
            case rocketHach_1:
            controller.setSetpoint(1.3);
            break;
            case rocketHach_2:
            controller.setSetpoint(1.2);
            break;
            case rocketBall_1:
            controller.setSetpoint(1.1);
            break;
            default:
    }
    }
    protected enum point{
        cagoHach,
        cagoBall,
        rocketHach_1,
        rocketHach_2,
        rocketBall_1,
        rocketBall_2
    }
}
