package frc.robot;

public class Const{

	//RobotMap
	public static final int joystickPort = 0;

	public static final int driveRightFrontPort;
	public static final int driveRightBackPort;
	public static final int driveLeftFrontPort;
	public static final int driveLeftBackPort;
	
	public static final int liftPort;

	public static final int barSolenoidPort;
	public static final int rollerMotorPort;

	public static final int armSolenoidPort;

	public static final int driveRightEnocderPort;
	public static final int driveLeftEncoderPort;
	public static final int liftEncoderPort;

	//constants
	public static final double shipCargoHeight;
	public static final double rocketCargo_1Height;
	public static final double rocketCargo_2Height;
	public static final double hatch_1Height;
	public static final double hatch_2Height;
	//enumとどっち？

	public static final enum armHeight{
		shipCargoHeight(),
		rocketCargoHeight(),
		rocketCargo_2Height(),
		hatch_1Height(),
		hatch_2Height();		

		public double height;
		private armHeight(double height){
			this.height = height;
		}
	}

	public static final double deadband = 0.1;

	public static final double kp_straight = 0;
	public static final double ki_straight = 0;
	public static final double kd_straight = 0;
	public static final double kp_rotate = 0;
	public static final double ki_rotate = 0;
	public static final double kd_rotate = 0;
	public static final double kp_lift = 0;
	public static final double ki_lift = 0;
	public static final double kd_lift = 0;
	
	public static final double PIDPeriod = 0.05;
	public static final double maxAcceleration = 0.5;
}
