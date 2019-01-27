/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class Drive extends DifferentialDrive{

    enum PIDMode {
		Straight, Rotate, Default
	};

	private double straightOutput, rotateOutput;
	private double preStraightOutput,preRotateOutput;

	private EncoderGroup e_drive;
	private ADXRS450_Gyro g_drive;

	private PIDController straightController, turnController;
	
	
 
	public Drive(SpeedController leftMotor, SpeedController rightMotor, EncoderGroup e_drive, ADXRS450_Gyro g_drive) {
		super(leftMotor, rightMotor);
		this.e_drive = e_drive;
		this.g_drive = g_drive;

		straightController = new PIDController(Const.kp_straight, Const.ki_straight, Const.kd_straight, e_drive,
				new DrivePIDOutput(PIDMode.Straight));
		turnController = new PIDController(Const.kp_rotate, Const.ki_rotate, Const.kd_rotate, g_drive,
				new DrivePIDOutput(PIDMode.Rotate));
	}

	

	public void setRelativeStraightSetpoint(double setpoint) {
		setRelativeSetpoint(e_drive.getDistance() + setpoint, g_drive.getAngle());
	}

	public void setRelativeTurnSetpoint(double setpoint) {
		setRelativeSetpoint(e_drive.getDistance(), g_drive.getAngle() + setpoint);
	}

	public void setRelativeSetpoint(double straightSetpoint, double turnSetpoint) {
		straightController.setSetpoint(e_drive.getDistance() + straightSetpoint);
		turnController.setSetpoint(g_drive.getAngle() + turnSetpoint);
    }
    
    public void setStraightSetpoint(double straightSetpoint){
        setSetpoint(straightSetpoint, 0);
    }

    public void setTurnSetpoint(double turnSetpoint){
        setSetpoint(0, turnSetpoint);
    }

    public void setSetpoint(double straightSetpoint, double turnSetpoint){
        straightController.setSetpoint(straightSetpoint);
		turnController.setSetpoint(turnSetpoint);
    }


	public void PIDEnable() {
		if (!straightController.isEnabled()) {
			straightController.enable();
		}
		if (!turnController.isEnabled()) {
			turnController.enable();
		}
	}

	public void PIDDisable() {
		if (straightController.isEnabled()) {
			straightController.disable();
		}
		if (turnController.isEnabled()) {
			turnController.disable();
		}
	}

	public boolean is_PIDEnabled() {
		return straightController.isEnabled() && turnController.isEnabled();
	}

	public class DrivePIDOutput implements PIDOutput {

		PIDMode m_pid = PIDMode.Default;

		DrivePIDOutput(PIDMode m_pid) {
			this.m_pid = m_pid;
		}

		public void pidWrite(double output) {

			switch (m_pid) {
			case Straight:
				straightOutput = output;
				break;

			case Rotate:
				rotateOutput = output;
				break;

			case Default:
			default:
			}
			straightOutput = LimitAcceleraton(preStraightOutput, straightOutput);
			rotateOutput = LimitAcceleraton(preRotateOutput, rotateOutput);

			arcadeDrive(-straightOutput, rotateOutput);

			preStraightOutput = straightOutput;
			preRotateOutput = rotateOutput;
		}

		private double LimitAcceleraton(double preOutput, double output ){
			if(preOutput == output) return output;
			double accelration = (output - preOutput) / Const.PIDPeriod;
			double Output = preOutput + Math.min(accelration, Const.maxAcceleration) * Const.PIDPeriod;
			
			return Math.min(1.0, Math.max(Output, -1.0));
		}
	}


}

/*　関数一覧
Drive(SpeedController leftMotor, SpeedController rightMotor, EncoderGroup e_drive, ADXRS450_Gyro g_drive)
	DifferentialDrive要素のSpeedController、PIDSource用のEnocoderGroupとADXRS450_Gyroを受け取り、PIDControllerのインスタンスを作る

setRelativeStraightSetpoint(double setpoint)
setRelativeTurnSetpoint(double setpoint)
setRelativeSetpoint(double straightSetpoint, double turnSetpoint)
	Setpointを相対位置で代入する
	 * 本来のsetSetPointでは現在位置からではなくEncoder系を作動させた時の初期位置からの距離、角度を指定する必要がある。
	 * PIDControllerのcalculate()をみるとinputにpidGet()をいれ、setSetpoint()でsetされた目標値との偏差をとっている。
	 * つまり、何も考えずに入れるとEncoder系を作動させた時の初期位置からのsetpointになり、思うように動作しない。
	 * 例えば、1ｍ前進した後に2ｍ後進しようとしてsetSetpoint(-2000)などとすると実際には3ｍ後進してしまう。
	 * だから、setSetpoint(getDistance()(=1000) + setpoint(=-2000))とする。
	 * しかし、少し扱いづらいので"Relative"(=相対)にして見えないところで処理する。
	 
setStraightSetpoint(double straightSetpoint)
setTurnSetpoint(double turnSetpoint)
setSetpoint(double straightSetpoint, double turnSetpoint)
	Setpointを絶対座標で代入する

PIDEnable()
	二つのPIDControllerをenableにする
PIDDisable()
	二つのPIDControllerをdisableにする
is_PIDEnabled()
	二つのPIDControllerがenableかどうか

*/
/*メンバークラス---DrivePIDOutput　PIDOutput継承
  その関数 
	DrivePIDOutput(PIDMode pidmode);
		PIDModeによって後述のpidWrite()の動作が変わる。

	pidWrite(double output);
		outputを受け取り処理して代入する。PIDModeによって代入対象が異なる。
*/


/*
追加で
setStraightP,I,D(double p,i,d);
setTurnP,I,D(double p,i,d);
    ゲインの調整をしやすくするため

isMoving();
	進んだり回ったりしているか
	Encoderのしきい値よく考える。

PIDReset();
	PIDのリセット

*/
