package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class State {

    public static enum DriveState {
        kManual,
        kLineTrace,
        kCloseToLine
    }

    public static enum CargoState {
        kHold,
        kRelease,
        kDoNothing
    }

    // Drive
    public DriveState driveState;
    public double driveStraightSpeed, driveRotateSpeed;
    public double driveStraightSetpoint, driveRotateSetpoint;    // PID制御の目標値
    public boolean is_drivePIDOn;    // PID制御するかどうか

    public boolean is_lineTraceOn;    // ライントレースするかどうか

    // Lift
    public double liftSpeed;    // コントローラー制御の値
    public double liftSetpoint;   // PID制御の目標値
    public boolean is_liftPIDOn;    // PID制御するかどうか

    // Grabber
    public CargoState cargoState;
    public boolean is_toHoldPanel;    // パネルを保持するかどうか
    public boolean is_toHoldArm;

    // Climb
    public boolean is_lockClimb;    // ストッパーを出すかどうか
    public double climbMotorSpeed;    // クライムの時の後輪のモーターの値

    public State() {
        stateInit();
    }

    public void stateInit(){

        // Drive
        driveState = DriveState.kManual;
        driveStraightSpeed = 0;
        driveRotateSpeed = 0;
        is_drivePIDOn = false;
        is_lineTraceOn = false;

        // Lift
        liftSpeed = 0;
        is_liftPIDOn = false;

        // Grabber
        cargoState = CargoState.kDoNothing;
        is_toHoldPanel = false;
        is_toHoldArm = false;
   
        // Climb
        is_lockClimb = false;
        climbMotorSpeed = 0;
    }

    public void printVariables(){
        SmartDashboard.putNumber("driveStraightSpeed", driveStraightSpeed);
    }

}
