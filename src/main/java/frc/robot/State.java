package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class State {

    public enum GrabberState {

    }

    // Drive
    public double driveXSpeed, driveYSpeed;  // コントローラー制御の値
    public double driveStraightSetpoint, driveTurnSetpoint; // PID制御の目標値
    public boolean is_drivePIDOn;  // PID制御するかどうか

    public boolean is_lineTraceOn;  // ライントレースするかどうか

    // Lift
    public double liftSpeed;    // コントローラー制御の値
    public double liftSetpoint; // PID制御の目標値
    public boolean is_liftPIDOn;    // PID制御するかどうか

    // Grabber
    public boolean whetherHoldCargo;    // カーゴを回収するかどうか
    public boolean whetherReleaseCargo; // カーゴを射出するかどうか
    public boolean whetherHoldPanel;    // パネルを保持するかどうか

    // Climb
    public boolean is_climbSolenoidOn;  // ストッパーを出すかどうか
    public double climbMotorSpeed;  // クライムの時の後輪のモーターの値
  
    // Commands
    public boolean is_noCommand;    // コマンドが入力されたかどうか
    public Const.Command command = Const.Command.noCommand;

    public void stateInit(){

        // Drive
        driveXSpeed = 0;
        driveYSpeed = 0;
        is_drivePIDOn = false;
        is_lineTraceOn = false;

        // Lift
        liftSpeed = 0;
        is_liftPIDOn = false;

        // Grabber
        whetherHoldCargo = false;
        whetherReleaseCargo = false;
        // パネルは押したら変わる制なので初期化はいらない
   
        // Climb
        is_climbSolenoidOn = false;
        climbMotorSpeed = 0;
  
        // Commands
        is_noCommand = true;// 入力がなかったらfalseにする。
    }

    public void printVariables(){
        SmartDashboard.putNumber("driveXSpeed",driveXSpeed);
    }

}