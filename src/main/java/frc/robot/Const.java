package frc.robot;

public class Const {
	/**
		Ports
	 */
	// Joystick
	public static final int JoystickPort = 0;
	public static final int DriveControllerPort = 0;
	public static final int OperateControllerPort = 1;
	
	// DriveBaseMotors
	public static final int DriveRightPort = 1;
	public static final int DriveLeftPort = 0;
	
	// LiftMotors
	public static final int LiftMotorPort = 2;
	public static final double KeepLiftHeightSpeed = 0;

	// ArmSolenoids & Motors
	public static final int ArmSolenoidPort = 0;
	public static final int BarSolenoidPort = 1;
	public static final int RollerMotorPort = 3;

	// Solenoids & Motors for climbing
	public static final int ClimbSolenoidPort = 2;
	public static final int ClimbMotorPort = 4;
	
	// DriveBaseEncoders
	public static final int RightDriveEncoderAPort = 0;
	public static final int RightDriveEncoderBPort = 1;
	public static final int LeftDriveEncoderAPort = 2;
	public static final int LeftDriveEncoderBPort =3;

	// LiftEncoders
	public static final int LiftEncoderAPort = 4;
	public static final int LiftEncoderBPort = 5;

	// Senosors for line trace
	public static final int RightFrontSensorPort = 0;
	public static final int RightBackSensorPort = 0;
	public static final int LeftFrontSensorPort = 0;
	public static final int LeftBackSensorPort = 0;
	
	/**
		Field Dimension	
		everything in cm
	 */
	public static final double CmPerInch = 2.54;

	// 穴の中央の高さ 
	public static final double RocketFirstCargoHeight = 26.8 * CmPerInch;
	public static final double RocketSecondCargoHeight = 54.8 * CmPerInch;
	public static final double RocketFirstHatchHeight = 18.3 * CmPerInch;
	public static final double RocketSecondHatchHeight = 46.3 * CmPerInch;
	public static final double ShipCargoHeight = 40 * CmPerInch;
	public static final double ShipHatchHeight = 18.3 * CmPerInch;
	public static final double HabSecondHeight = 6 * CmPerInch + 10;
	public static final double HabThirdHeight = 19 * CmPerInch;
	public static final double GroundHeight = 0;


	/**
		Robot Dimension
	 */
	public static final double DriveEncoderDistancePerPulse = 7.7 * Math.PI / 10.71;
    public static final double LiftMinHeight = 0 ;
	public static final double LiftEncoderDistancePerPulse = -0.14;
	
	public static final double LiftPIDTolearnce = 5;
	
	// NetworkTable for finding lines
	public static final String LineFindNetworkTable = "";

	public static enum ArmHeight {
		shipCargoHeight(0),
		rocketFirstCargoHeight(0),
		rocketSecondCargoHeight(0),
		panel_1Height(0),
		panel_2Height(0);		

		public double height;
		private  ArmHeight(double height) {
			this.height = height;
		}
	}

	// Deadband
	public static final double Deadband = 0.1;

	// Constants for PID control
	public static final double DriveStraightKp = 0;
	public static final double DriveStraightKi = 0;
	public static final double DriveStraightKd = 0;
	public static final double DriveRotateKp = 0;
	public static final double DriveRotateKi = 0;
	public static final double DriveRotateKd = 0;
	public static final double LiftKp = 0.06;
	public static final double LiftKi = 0;
	public static final double LiftKd = 0;

	// Constants for limitting acceleration
	public static final double PIDPeriod = 0.05;
	public static final double maxAcceleration = 0.5;

	// Constant for sleep after Solenid moving
	public static final int SolenoidSleepTime = 100;

	//Constants for getting distance to the line
	public static final double Theta_Camera_rad = Math.toRadians(0);
	public static final double Theta_Angle_rad = Math.toRadians(0);
	public static final double cameraHeight = 0;
	
}
