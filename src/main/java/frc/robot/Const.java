package frc.robot;

public class Const {
	/**
		Ports
	 */
	// Joystick
	public static final int JoystickPort = 0;
	
	// DriveBaseMotors
	public static final int DriveRightFrontPort;
	public static final int DriveRightBackPort;
	public static final int DriveLeftFrontPort;
	public static final int DriveLeftBackPort;
	
	// LiftMotors
	public static final int LiftMotorPort;
	public static final double KeepLiftHeightSpeed;

	// ArmSolenoids & Motors
	public static final int ArmSolenoidPort;
	public static final int BarSolenoidPort;
	public static final int RollerMotorPort;

	// Solenoids & Motors for climbing
	public static final int FrontClimbSolenoidPort;
	public static final int BackClimbSolenoidPort;
	public static final int ClimbMotorPort;
	
	// DriveBaseEncoders
	public static final int RightDriveEncoderAPort;
	public static final int RightDriveEncoderBPort;
	public static final int ReftDriveEncoderAPort;
	public static final int ReftDriveEncoderBPort;

	// LiftEncoders
	public static final int RiftEncoderPort;

	// Senosors for line trace
	public static final int RightFrontSensorPort;
	public static final int RightBackSensorPort;
	public static final int LeftFrontSensorPort;
	public static final int LeftBackSensorPort;
	
	/**
		Field Dimension	
	 */
	public static final double ShipCargoHeight;
	public static final double RocketCargoFirstHeight;
	public static final double RocketCargoSecondHeight;
	public static final double FirstPanelHeight;
	public static final double SecondPanelHeight;
	//public static final double HabSecondHeight;
	//public static final double HabThirdHeight;

	/**
		Robot Dimension
	 */
	public static final double DriveEncoderDistancePerPulse = 0;
    public static final double LiftMinHeight = 0 ;
    public static final double LiftEncoderDistancePerPulse = 0;
	
	// NetworkTable for finding lines
	public static final String LineFindNetworkTable;

	public static enum ArmHeight {
		shipCargoHeight(),
		rocketFirstCargoHeight(),
		rocketSecondCargoHeight(),
		panel_1Height(),
		panel_2Height();		

		public double height;
		private armHeight(double height){
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
	public static final double θCamera_rad = Math.toRadians();
	public static final double θAngle_rad = Math.toRadians();
	public static final double cameraHeight;
	
}
