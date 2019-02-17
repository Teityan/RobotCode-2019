/*---------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.          
                                                     */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Lift {
    /*
    <モーター>
    モーターの速度を変更する
    set数字);
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
    Talon liftMotor;
    PIDController pid;
    PIDMotor motor;

    public static enum PredefinedSetpoint {
        shipHatch(1.5), 
        shipCargo(1.4), 
        rocketHatch_1(1.3), 
        rocketHatch_2(1.2),
        rocketCargo_1(1.1),
        rocketCargo_2(1.0);

        private final double setpoint;

        private PredefinedSetpoint(final double setpoint) {
            this.setpoint = setpoint;
        }
        public double getSetpoint() {
            return this.setpoint;
        }
    }

    // コンストラクター
    Lift(Talon liftMotor, Encoder encoder) {
        this.encoder = encoder;
        this.liftMotor = liftMotor;
        this.motor = new PIDMotor(liftMotor , this);
        this.pid = new PIDController(Const.LiftKp, Const.LiftKi, Const.LiftKd, encoder, motor);
        pid.setAbsoluteTolerance(Const.LiftPIDTolearnce);
    }

	/**
	 * ToDo
	 */
	public void applyState(State state) {
        // PIDが有効か調べる
        if(state.is_liftPIDOn){
            this.setSetpoint(state.liftSetpoint);
            this.enablePID();
        }else{
            this.disablePID();
            this.setSpeed(state.liftSpeed);
        }
	}

    // モーター
    public void setSpeed(double speed) {
        liftMotor.set(speed);
    }

    // エンコーダー
    public double getHeight() {
        double height = encoder.getDistance();
        return height;
    }
    
    public double getSpeed() {
        double speed = encoder.getRate();
        return speed;
    }

    // PID
    public void enablePID() {
        pid.enable();
    }

    public void disablePID() {
        pid.disable();
    }

    public void setSetpoint(double height) {
        pid.setSetpoint(height);
    }

    public void setSetopoint(PredefinedSetpoint point) {
        pid.setSetpoint(point.getSetpoint());
    }

    public boolean is_PIDOnTarget() {
        if(!(pid.getSetpoint() == 0)){
            if(pid.isEnabled()) {
                return pid.onTarget();
            }
        }
        // PIDがEnableにされてなかったら判断しない
        // PIDのSetpointが0だったら判断しない
        return false;
        
    }

    public void printVariables() {
        SmartDashboard.putNumber("getHeight()", getHeight());
        SmartDashboard.putNumber("pid.getP()", pid.getP());
        SmartDashboard.putNumber("pid.getI()", pid.getI());
        SmartDashboard.putNumber("pid.getD()", pid.getD());

    }
}
class PIDMotor implements PIDOutput {
    
    public Talon motor;
    public Lift lift;

    public PIDMotor(Talon motor, Lift lift) {
        this.motor = motor;
        this.lift = lift;
    }

    public void pidWrite(double output) {
        lift.setSpeed(output);
    }
}