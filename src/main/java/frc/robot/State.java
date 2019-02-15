package frc.robot;

import java.awt.image.SampleModel;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class State {

    public enum DriveState {
        kManual,
        kLineTrace,
        kCloseToLine
    }

    public enum CargoState {
        kHold,
        kRelease,
        kDoNothing
    }

    public enum ClimbSequence {
        kDoNothing,
        kLiftUp,
        kLocking,
        kLiftDown
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
    public boolean is_toRetractArm;

    // Climb
    public ClimbSequence climbSequence = ClimbSequence.kLiftUp;
    public boolean is_autoClimbOn;
    public boolean is_lockClimb;    // ストッパーを出すかどうか
    public double climbMotorSpeed;    // クライムの時の後輪のモーターの値

    public State() {
        stateInit();
    }

    public void stateInit() {

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
        is_toRetractArm = false;
   
        // Climb
        // climbSequence = ClimbSequence.kDoNothing;
        is_autoClimbOn = false;
        is_lockClimb = false;
        climbMotorSpeed = 0;
    }

    public void printVariables() {
        SmartDashboard.putNumber("ddriveStraightSpeed", driveStraightSpeed);
        SmartDashboard.putNumber("driveRotateSpeed", driveRotateSpeed);
        SmartDashboard.putNumber("ddriveStraightSetpoint", driveStraightSetpoint);
        SmartDashboard.putNumber("driveRotateSetpoint", driveRotateSetpoint);
        SmartDashboard.putNumber("liftSpeed", liftSpeed);
        SmartDashboard.putNumber("liftSetpoint", liftSetpoint);
        SmartDashboard.putNumber("climbMotorSpeed", climbMotorSpeed);

        SmartDashboard.putBoolean("is_drivePIDOn", is_drivePIDOn);
        SmartDashboard.putBoolean("is_lineTraceOn", is_lineTraceOn);
        SmartDashboard.putBoolean("is_liftPIDOn", is_liftPIDOn);
        SmartDashboard.putBoolean("is_toHoldPanel", is_toHoldPanel);
        SmartDashboard.putBoolean("is_toRetractArm", is_toRetractArm);
        SmartDashboard.putBoolean("is_autoClimbOn;", is_autoClimbOn);
        SmartDashboard.putBoolean("is_lockClimb;", is_lockClimb);
    }
}
