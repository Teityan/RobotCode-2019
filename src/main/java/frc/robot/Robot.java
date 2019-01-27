/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Robot extends TimedRobot {
//Controller
  private Joystick joystick;

//Motor
  private  driveRightFront, driveRightBack, driveLeftFront, driveLeftBack;
  private  liftMotor;
  private  rollerMotor;

//SpeedControllerGroup
  private SpeedControllerGroup rightDriveGroup, leftDriveGroup;

//Encoder,Gyro
  private Encoder rightDriveEncoder, leftDriveEncoder;
  private EncoderGroup driveEncoder;
  private Encoder liftEncoder;
  private ADXRS450_Gyro gyro;

//Solenoid
  private Solenoid armSolenoid, barSolenoid;
  private Solenoid frontClimbSolenoid, backClimbSolenoid;

//PIDController
  private Drive drive;
  private Lift lift;

//Grabber
  private Grabber grabber;

//Sensor
  private AnalogInput rightFrontSensor, 
                      rightBackSensor, 
                      leftFrontSensor, 
                      leftBackSensor;

//Camera
  private CameraServer camera;

//NetWorkTable
  private NetworkTable networkTable;


//variables
  //Drive
    private double driveXValue, driveYValue;


  //Lift
    private double liftValue;


  //Grabber
    private boolean whetherHoldCargo;
    private boolean whetherReleaseCargo;
    private boolean whetherHoldPanel;
    private boolean lastWhetherHoldPanel = false;
  
  //Command
    private boolean is_commandInput;
    private enum Command{
      movetoShipCargoHeight,
      movetoRocketCargo_1Height,
      movetoRocketCargo_2Height,
      movetoPanel_1Height,
      movetoPanel_2Height,
      lineTrace,
      noCommand;
    }
    private Command command = Command.noCommand;

//functions
  private double deadbandProcessing(double value){
    return value * Math.abs(value) > Const.deadband ? 1 : 0 ;
  }

  @Override
  public void robotInit() {
    //Controller
    joystick = new Joystick(Const.joystickPort);


    //Motor
    driveRightFront  = new (Const.driveRightFrontPort);
    driveRightBack = new (Const.driveRightBackPort);
    driveLeftFront = new (Const.driveLeftFrontPort);
    driveLeftBack = new (Const.driveLeftBackPort);

    liftMotor = new (Const.liftMotorPort);

    rollerMotor = new (Const.rollerMotorPort);

    //SpeedControllerGroup
    rightDriveGroup = new SpeedControllerGroup(Const.driveRightFront, Const.driveRightBack);
    leftDriveGroup = new SpeedControllerGroup(Const.driveLeftFront, Const.driveLeftBack);

    //Encoder,Gyro
    rightDriveEncoder = new Enocoder(Const.rightDriveEncoderAPort, Const.rightDriveEncoderBPort);
    leftDriveEncoder = new Enocder(Cosnt.leftDriveEncoderAPort, Const.leftDriveEncoderBPort);
    driveEncoder = new EncoderGroup(rightDriveEncoder, leftDriveEncoder);

    liftEncoder = new Encoder(Const.liftEncoderPort);

    gyro = new ADXRS450_Gyro();

    //Solenoid
    armSolenoid = new Solenoid(Const.armSolenoidPort);
    barSolenoid = new Solenoid(Const.barSolenoidPort);

    frontClimbSolenoid = new Solenoid(Const.frontClimbSolenoidPort);
    backClimbSolenoid = new Solenoid(Const.backClimbSolenoidPort);

    //PIDController
    drive = new Drive(rightDriveGroup, leftDriveGroup, driveEncoder, gyro);
    lift = new Lift(liftMotor, liftEncoder);

    //Grabber
    grabber = new Grabber(rollerMotor, barSolenoid, armSolenoid);

    //Sensor
    rightFrontSensor = new AnalogInput(Const.rightFrontSensorPort);
    rightBackSensor = new AnalogInput(Const.rightBackSensorPort);
    leftFrontSensor = new AnalogInput(Const.leftFrontSensor);
    leftBackSensor = new AnalogInput(Const.leftBackSensor);

    //Camera
    camera = camera.getInstance();
    camera.startAutomaticCapture();

    //NetworkTable
    networkTable = networkTable.getSubTable(Const.lineFindNetworkTable);
    
  }
  
  @Override
  public void autonomousInit() {
   
  }

  
  @Override
  public void autonomousPeriodic() {
    
  }

  @Override
  public void teleopInit() {
    drive.PIDReset();
    lift.PIDReset();
  }
  
  @Override
  public void teleopPeriodic() {

    //Command
    {
      is_commandInput = true;

      command = dertermineCommand();

      switch(command){


        case noCommand:
          is_commandInput = false;
        default:
      }     


    }

    if(!is_commandInput){
    //input
      driveXValue = deadbandProcessing(joystick.getX());
      driveYValue = deadbandProcessing(joystick.getY());

      liftValue = deadbandProcessing(joystick.getThrottle());

      whetherHoldCargo = joystick.getRawButton();

      if(joystick.getRawButton()){
        whetherHoldPanel = !whetherHoldPanel;//ボタンを押したら状態が変わる
      }
    
    //Substitute
      drive.arcadeDrive(driveXValue, driveYValue);

      lift.setSpeed(liftValue);
      
      if(whetherHoldCargo){
        grabber.holdCargo();
      }else{
        grabber.stopRoller();
      }

      if(whetherReleaseCargo){
        grabber.releaseCargo();
      }else{
        grabber.stopRoller();
      }
      
      if(whetherHoldPanel){
        grabber.holdPanel();
      }else{
        grabber.releasePanel();
      }
    }
  }
}
