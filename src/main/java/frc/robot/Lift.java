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

    public static enum Point {
        shipHatch(1.5), 
        shipCargo(1.4), 
        rocketHatch_1(1.3), 
        rocketHatch_2(1.2),
        rocketCargo_1(1.1),
        rocketCargo_2(1.0);

    private final double point;

    private Point(final double point) {
        this.point = point;
    }
    public double getHeight() {
        return this.point;
    }
    }

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

    public void setSetopoint(Point point){
        switch(point){
            case shipHatch:
            controller.setSetpoint(Point.shipHatch.getHeight());
            break;
            case shipCargo:
            controller.setSetpoint(Point.shipCargo.getHeight());
            break;
            case rocketHatch_1:
            controller.setSetpoint(Point.rocketHatch_1.getHeight());
            break;
            case rocketHatch_2:
            controller.setSetpoint(Point.rocketHatch_2.getHeight());
            break;
            case rocketCargo_1:
            controller.setSetpoint(Point.rocketCargo_1.getHeight());
            case rocketCargo_2:
            controller.setSetpoint(Point.rocketCargo_2.getHeight());
            break;
            default:
    }        
    }

}
