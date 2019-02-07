package frc.robot;

public class Const{

	//Joystick
	public static final int joystickPort = 0;

	//DriveBaseMotors
	public static final int driveRightFrontPort;
	public static final int driveRightBackPort;
	public static final int driveLeftFrontPort;
	public static final int driveLeftBackPort;
	
	//LiftMotors
	public static final int liftMotorPort;
	public static final double keepLiftHeightSpeed;

	//ArmSolenoids & Motors
	public static final int armSolenoidPort;
	public static final int barSolenoidPort;
	public static final int rollerMotorPort;

	//Solenoids & Motors for climbing
	public static final int frontClimbSolenoidPort;
	public static final int backClimbSolenoidPort;
	public static final int climbMotorPort;
	
	//DriveBaseEncoders
	public static final int rightDriveEncoderAPort;
	public static final int rightDriveEncoderBPort;
	public static final int leftDriveEncoderAPort;
	public static final int leftDriveEncoderBPort;

	//LiftEncoders
	public static final int liftEncoderPort;

	//Senosors for line trace
	public static final int rightFrontSensorPort;
	public static final int rightBackSensorPort;
	public static final int leftFrontSensorPort;
	public static final int leftBackSensorPort;
	
	//NetworkTable for finding lines
	public static final String lineFindNetworkTable;

	//Height
	public static final double shipCargoHeight;
	public static final double rocketCargo_1Height;
	public static final double rocketCargo_2Height;
	public static final double panel_1Height;
	public static final double panel_2Height;

	public static final double hab_2Height;
	public static final double hab_3Height;
	//enumとどっち？

	public static enum armHeight{
		shipCargoHeight(),
		rocketCargo_1Height(),
		rocketCargo_2Height(),
		panel_1Height(),
		panel_2Height();		

		public double height;
		private armHeight(double height){
			this.height = height;
		}
	}

	public enum Command{
	//Drive
	  closeToLine,
	  lineTrace,
	
	//Lift
	  moveToShipCargoHeight,
	  moveToRocketCargo_1Height,
	  moveToRocketCargo_2Height,
	  moveToPanel_1Height,
	  moveToPanel_2Height,

	  keepLiftHeight,

	//Arm
	  holdCargo,
	  releaseCargo,
	  changeBarState,
	
	//Climb
	  climbMoveToHab_2Height,
	  climbMoveToHab_3Height,
	  climbStopperOn,
	  climbLiftDown,
	  climbAdvance,
	
	  noCommand;
	}

	//deadband
	public static final double deadband = 0.1;

	//Constants for PID control
	public static final double kp_straight = 0;
	public static final double ki_straight = 0;
	public static final double kd_straight = 0;
	public static final double kp_rotate = 0;
	public static final double ki_rotate = 0;
	public static final double kd_rotate = 0;
	public static final double kp_lift = 0;
	public static final double ki_lift = 0;
	public static final double kd_lift = 0;
	
	//Constants for limitting acceleration
	public static final double PIDPeriod = 0.05;
	public static final double maxAcceleration = 0.5;

	//Constants for getting distance to the line
	public static final double θCamera_rad = Math.toRadians();
	public static final double θAngle_rad = Math.toRadians();
	public static final double cameraHeight;
	
}
