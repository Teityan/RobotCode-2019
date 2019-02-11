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
	public static final int DriveRightFrontPort = 0;
	public static final int DriveRightBackPort = 0;
	public static final int DriveLeftFrontPort = 0;
	public static final int DriveLeftBackPort = 0;
	
	// LiftMotors
	public static final int LiftMotorPort = 0;
	public static final double KeepLiftHeightSpeed = 0;

	// ArmSolenoids & Motors
	public static final int ArmSolenoidPort = 0;
	public static final int BarSolenoidPort = 0;
	public static final int RollerMotorPort = 0;

	// Solenoids & Motors for climbing
	public static final int FrontClimbSolenoidPort = 0;
	public static final int BackClimbSolenoidPort = 0;
	public static final int ClimbMotorPort = 0;
	
	// DriveBaseEncoders
	public static final int RightDriveEncoderAPort = 0;
	public static final int RightDriveEncoderBPort = 0;
	public static final int LeftDriveEncoderAPort = 0;
	public static final int LeftDriveEncoderBPort = 0;

	// LiftEncoders
	public static final int LiftEncoderAPort = 0;
	public static final int LiftEncoderBPort = 0;

	// Senosors for line trace
	public static final int RightFrontSensorPort = 0;
	public static final int RightBackSensorPort = 0;
	public static final int LeftFrontSensorPort = 0;
	public static final int LeftBackSensorPort = 0;
	
	/**
		Field Dimension	
	 */
	public static final double ShipCargoHeight = 0;
	public static final double RocketCargoFirstHeight = 0;
	public static final double RocketCargoSecondHeight = 0;
	public static final double FirstPanelHeight = 0;
	public static final double SecondPanelHeight = 0;
	//public static final double HabSecondHeight;
	//public static final double HabThirdHeight;

	/**
		Robot Dimension
	 */
	public static final double DriveEncoderDistancePerPulse = 0;
    public static final double LiftMinHeight = 0 ;
    public static final double LiftEncoderDistancePerPulse = 0;
	
	// NetworkTable for finding lines
	public static final String LineFindNetworkTable = "";

	public static enum ArmHeight {
		shipCargoHeight(0),
		rocketFirstCargoHeight(0),
		rocketSecondCargoHeight(0),
		panel_1Height(0),
		panel_2Height(0);		

		public double height;
		private  ArmHeight(double height){
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
	public static final double LiftKp = 0;
	public static final double LiftKi = 0;
	public static final double LiftKd = 0;

	// Constants for limitting acceleration
	public static final double PIDPeriod = 0.05;
	public static final double maxAcceleration = 0.5;

	//Constants for getting distance to the line
	public static final double Theta_Camera_rad = Math.toRadians(0);
	public static final double Theta_Angle_rad = Math.toRadians(0);
	public static final double cameraHeight = 0;
	
}
