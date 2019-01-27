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

//Motors
  private  driveRightFront, driveRightBack, driveLeftFront, driveLeftBack;
  private  liftMotor;
  private  rollerMotor;
  private  climbMotor;

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
  
  //Commands
    private boolean is_commandInput;
    private Const.Command command = Const.Command.noCommand;

//functions
  private double deadbandProcessing(double value){
    return value * Math.abs(value) > Const.deadband ? 1 : 0 ;
  }

  private Const.Command getCommand(){
    
  }
@Override
public void robotInit() {
    //Controller
    joystick = new Joystick(Const.joystickPort);


    //Motors
    driveRightFront  = new (Const.driveRightFrontPort);
    driveRightBack = new (Const.driveRightBackPort);
    driveLeftFront = new (Const.driveLeftFrontPort);
    driveLeftBack = new (Const.driveLeftBackPort);

    liftMotor = new (Const.liftMotorPort);

    rollerMotor = new (Const.rollerMotorPort);

    climbMotor = new (Const.climbMotorPort);

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
   armSolenoid.set(true);//しまってあるアームを展開する
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
  /*Init
   * これからコマンドやコントローラーで変数の値を変えてから代入する。
   * 操作しないときは出力を出したくないため、最初に出力を出さない状態で初期化する。
   */

  /*Command
   * コントローラーからコマンドを受け取りそれに応じて変数の値を変える。
   */

    is_commandInput = true;//入力がなかったらfalseにする。
    command = getCommand();//コマンドを判別

    switch(command){
    //Drive
      case closeToLine:

      case lineTrace:

    /*Lift
     *  PID制御ならliftSetpointに代入、is_PIDOnをtrueにする。
     *  モーターの値を制御するならliftValueに代入
     */
      case moveToShipCargoHeight:
        liftSetpoint = Const.shipCargoHeight;
        is_liftPIDOn = true;
        break;

      case moveToRocketCargo_1Height:
        liftSetpoint = Const.rocketCargo_1Height;
        is_liftPIDOn = true;
        break;

      case moveToRocketCargo_2Height:
        liftSetpoint = Const.rocketCargo_2Height;
        is_liftPIDOn = true;
        break;

      case moveToPanel_1Height:
        liftSetpoint = Const.panel_1Height;
        is_liftPIDOn = true;
        break;

      case moveToPanel_2Height:
        liftSetpoint = Const.panel_2Height;
        is_liftPIDOn = true;
        break;

      case keepLift:
        liftValue = Const.keepLiftSpeed;
        break;

    //Arm
     
      case holdCargo:
        whetherHoldCargo = true;
        break;

      case releaseCargo:
        whetherReleaseCargo = true;
        break;

      case changeBarState:
        whetherHoldPanel = !whetherHoldPanel;//ボタンを押したら状態が変わる
        break;

    /*Climb
     * Climbの流れ
     * 　1.リフトを上げる。
     * 　2.ソレノイドでストッパーを出す。
     *   3.リフトを下げる。(ここで車体が持ち上がる)
     *   4.モーターを回して前に進む
     */
      //1
      case cliimbMoveToHab_2Height:
        liftSetpoint = Const.hab_2Height;
        is_liftPIDOn = true;
        break;

      case climbMoveToHab_3Height:
        liftSetpoint = Const.hab_3Height;
        is_liftPIDOn = true;
        break;

      //2
      case climbStopperOn:
        is_climbSolenoidOn = true;
        break;

      //3
      case climbLiftDown:
        liftValue = -1.0;
        break;

      //4
      case climbAdvance:
        driveXValue = ;
        climbMotorValue = 1.0;
        break;




      case noCommand:
      default:
      is_commandInput = false;
    }     
  

  if(!is_commandInput){
  /*Controller
   * コマンドがなかった時にコントローラーから値を受け取り代入する。
   */
    driveXValue = deadbandProcessing(joystick.getX());
    driveYValue = deadbandProcessing(joystick.getY());

    liftValue = deadbandProcessing(joystick.getThrottle());
    
  }


  /*Substitute
   *  コマンドやコントローラーによって変えられた変数を代入する。
   */
    if(is_drivePIDOn){
      drive.setSetpoint(driveStraightSetpoint, driveTurnSetpoint);//PID制御
      drive.PIDEnable();
    }else{
      drive.PIDDisable();
      drive.arcadeDrive(driveXValue, driveYValue);//普通のモーター制御
    }


    if(is_liftPIDOn){
      lift.setSetpoint(liftSetpoint);//PID制御
      lift.PIDEnable();
    }else{
      lift.PIDDisable();
      lift.setSpeed(liftValue);//普通のモーター制御
    }

      
    if(whetherHoldCargo){
      grabber.holdCargo();//カーゴを回収
    }else{
      grabber.stopRoller();//ローラーを止める
    }

    if(whetherReleaseCargo){
      grabber.releaseCargo();//カーゴ射出
    }else{
      grabber.stopRoller();//ローラーを止める
    }
      
    if(whetherHoldPanel){
      grabber.holdPanel();//パネルをつかむ
    }else{
      grabber.releasePanel();//パネルを離す
    }
  
    frontClimbSolenoid.set(is_climbSolenoidOn);//前のベアリング付き爪を出す
    backClimbSolenoid.set(is_climbSolenoidOn);//後ろのモーター付き爪を出す
    climbMotor.set(climbMotorValue);//後ろのモーターを動かす。
  }
}
