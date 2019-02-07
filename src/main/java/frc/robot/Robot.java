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
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Const.Command;


public class Robot extends TimedRobot {
  // Controller
  private Joystick joystick;

  //Motors
  private Spark driveRightFront, driveRightBack, driveLeftFront, driveLeftBack;
  private Talon liftMotor;
  private Talon rollerMotor;
  private Talon climbMotor;

// SpeedControllerGroup
  private SpeedControllerGroup rightDriveGroup, leftDriveGroup;

// Encoder, Gyro
  private Encoder rightDriveEncoder, leftDriveEncoder;
  private EncoderGroup driveEncoder;
  private Encoder liftEncoder;
  private ADXRS450_Gyro gyro;

// Solenoid
  private Solenoid armSolenoid, barSolenoid;
  private Solenoid frontClimbSolenoid, backClimbSolenoid;

// SubModule
  private Drive drive;
  private Lift lift;
  private Grabber grabber;

// ライントレース用のセンサー　
// 0～3.3V 白線があると電圧が上がる 

  private AnalogInput rightFrontSensor, 
                      rightBackSensor, 
                      leftFrontSensor, 
                      leftBackSensor;

// Camera
  private CameraServer camera;

// NetWorkTable
  private NetworkTable networkTable;


// variables

  private State state;
  private double distanceToLine[] = new double[2], 
        displayPosition[] = new double[2];

// functions
  private double deadbandProcessing(double value) {
    return Math.abs(value) > Const.deadband ? value : 0 ;
  }

  private Const.Command getCommand() {
    // まずそれぞれのボタンの状態を把握
    boolean buttonPressed[] = new boolean[12];

    buttonPressed[0] = joystick.getTrigger(); 
    for(int i = 1; i<=12; i++){
      buttonPressed[i] = joystick.getRawButton(i);
    }
    
    /*ここで判定していく
    Joystickの場合
     *  3 Drive関係
     *  |__7 closeToLine   
     *  |__8 lineTrace
     * 
     *  4 Lift関係
     *  |__7 moveToShipCargoHeight
     *  |__8 moveToRocketCargo_1Height
     *  |__9 moveToRocketCargo_2Height
     *  |__10 moveToPanel_1Height
     *  |__11 moveToPanel_2Height
     *  |__12 keepLiftHeight
     * 
     *  5 Grabber関係
     *  |__7 holdCargo
     *  |__8 releaseCargo
     *  |__9 changeBarState
     *  
     *  6 Climb関係
     *  |__7 climbMoveToHab_2Height
     *  |__8 climbMoveToHab_3Height
     *  |__9 climbStopperOn
     *  |__10 climbLiftDown
     *  |__11 climbAdvance
     *  
     */

     if(buttonPressed[3]){
      if(buttonPressed[7])
      if(buttonPressed[8])
     }

     if(buttonPressed[4]){
      if(buttonPressed[7])
      if(buttonPressed[8])
      if(buttonPressed[9])
      if(buttonPressed[10])
      if(buttonPressed[11])
      if(buttonPressed[12])
     }

     if(buttonPressed[5]){
      if(buttonPressed[7])
      if(buttonPressed[8])
      if(buttonPressed[9])
     }

     if(buttonPressed[6]){
      if(buttonPressed[7])
      if(buttonPressed[8])
      if(buttonPressed[9])
      if(buttonPressed[10])
      if(buttonPressed[11])
     }
     
     return Command.noCommand;


  }

  public double[] geLinePosition(double[] displayPosition) {
    double θc = Const.θCamera_rad;
    double θa = Const.θAngle_rad;
    double maxH = Const.cameraHeight;

    // xは中央から右向き正  yは下から上向き正
    double x = displayPosition[0];
    double y = displayPosition[1];
    double result[] = new double[2]; 

       
    // double aboveMaxX = maxH * Math.tan(θa);
    // double minH = maxH - maxY * Math.cos(Math.PI/2 - (θc + θa));     
    double maxY = maxH / Math.cos(θc) * Math.sin(θa) *2;// 縦幅
        
    double distanceCamera_Display = Math.sqrt(y*y + maxH/Math.cos(θc)*maxH/Math.cos(θc) - 2*y*maxH*Math.sin(θa)/Math.cos(θc) );
    double maxX = 2 * distanceCamera_Display * Math.tan(θa);// 横幅
        

    double sinDistanceCamera_Display = y * Math.cos(θa) / distanceCamera_Display;
    double angleDisplay_LineOfSight  = Math.PI/2 - θa + Math.asin(sinDistanceCamera_Display);
    double l = y * Math.sin(angleDisplay_LineOfSight) / Math.sin(angleDisplay_LineOfSight + θc + θa);
    result[1] = maxH * Math.tan(θc) + l;
        
       
    
    double distanceCamera_DisplayZ = distanceCamera_Display * Math.cos(θc + angleDisplay_LineOfSight);
    result[0]  = maxH * x / distanceCamera_DisplayZ; 


    return result;
  }

@Override
public void robotInit() {
    // Controller
    joystick = new Joystick(Const.joystickPort);


    // Motors
    driveRightFront  = new Spark(Const.driveRightFrontPort);
    driveRightBack = new Spark(Const.driveRightBackPort);
    driveLeftFront = new Spark(Const.driveLeftFrontPort);
    driveLeftBack = new Spark(Const.driveLeftBackPort);

    liftMotor = new Talon(Const.liftMotorPort);

    rollerMotor = new Talon(Const.rollerMotorPort);

    climbMotor = new Talon(Const.climbMotorPort);

    // SpeedControllerGroup
    rightDriveGroup = new SpeedControllerGroup(Const.driveRightFront, Const.driveRightBack);
    leftDriveGroup = new SpeedControllerGroup(Const.driveLeftFront, Const.driveLeftBack);

    // Encoder,Gyro
    rightDriveEncoder = new Encoder(Const.rightDriveEncoderAPort, Const.rightDriveEncoderBPort);
    leftDriveEncoder = new Encoder(Const.leftDriveEncoderAPort, Const.leftDriveEncoderBPort);
    driveEncoder = new EncoderGroup(rightDriveEncoder, leftDriveEncoder);

    liftEncoder = new Encoder(Const.liftEncoderPort, );

    gyro = new ADXRS450_Gyro();

    // Solenoid
    armSolenoid = new Solenoid(Const.armSolenoidPort);
    barSolenoid = new Solenoid(Const.barSolenoidPort);

    frontClimbSolenoid = new Solenoid(Const.frontClimbSolenoidPort);
    backClimbSolenoid = new Solenoid(Const.backClimbSolenoidPort);

    // PIDController
    drive = new Drive(rightDriveGroup, leftDriveGroup, driveEncoder, gyro);
    //lift = new Lift(liftMotor, liftEncoder);

    // Grabber
    grabber = new Grabber(rollerMotor, barSolenoid, armSolenoid);

    // Sensor
    rightFrontSensor = new AnalogInput(Const.rightFrontSensorPort);
    rightBackSensor = new AnalogInput(Const.rightBackSensorPort);
    leftFrontSensor = new AnalogInput(Const.leftFrontSensorPort);
    leftBackSensor = new AnalogInput(Const.leftBackSensorPort);

    // Camera
    camera = camera.getInstance();
    camera.startAutomaticCapture();

    // NetworkTable
    networkTable = networkTable.getSubTable(Const.lineFindNetworkTable);

    // State
    state = new State();
    
  }
  
@Override
public void autonomousInit() {
   armSolenoid.set(true);// しまってあるアームを展開する
}

@Override
public void autonomousPeriodic() {
}

@Override
public void teleopInit() {
  drive.PIDReset();
  //lift.PIDReset();
}
  
@Override
public void teleopPeriodic() {
  /*Init
   * これからコマンドやコントローラーで変数の値を変えてから代入する。
   * 操作しないときは出力を出したくないため、最初に出力を出さない状態で初期化する。
   */
    state.stateInit();


  /*Command*
   * コントローラーからコマンドを受け取りそれに応じて変数の値を変える。
   */
    state.command = getCommand();// コマンドを判別


    switch(state.command){
    // Drive
      case closeToLine:
        distanceToLine = getLinePosition(displayPosition);
        state.driveStraightSetpoint = Math.sqrt(distanceToLine[0]*distanceToLine[0] + distanceToLine[1]*distanceToLine[1]);
        state.driveTurnSetpoint = Math.atan(distanceToLine[0]/distanceToLine[1]);
        break;

      case lineTrace:

        break;

    /*Lift
     *  PID制御ならliftSetpointに代入、is_PIDOnをtrueにする。
     *  モーターの値を制御するならliftSpeedに代入
     */
      case moveToShipCargoHeight:
        state.liftSetpoint = Const.shipCargoHeight;
        state.is_liftPIDOn = true;
        break;

      case moveToRocketCargo_1Height:
        state.liftSetpoint = Const.rocketCargo_1Height;
        state.is_liftPIDOn = true;
        break;

      case moveToRocketCargo_2Height:
        state.liftSetpoint = Const.rocketCargo_2Height;
        state.is_liftPIDOn = true;
        break;

      case moveToPanel_1Height:
        state.liftSetpoint = Const.panel_1Height;
        state.is_liftPIDOn = true;
        break;

      case moveToPanel_2Height:
        state.liftSetpoint = Const.panel_2Height;
        state.is_liftPIDOn = true;
        break;

      case keepLiftHeight:
        state.liftSpeed = Const.keepLiftHeightSpeed;
        break;

    /*Arm
     * ローラーやメカナムを回してカーゴを回収したり射出したりする
     * 棒を開いたり閉じたりしてパネルを保持したり開放したりする
     */
     
      case holdCargo:
        state.whetherHoldCargo = true;
        break;

      case releaseCargo:
        state.whetherReleaseCargo = true;
        break;

      case changeBarState:
        state.whetherHoldPanel = !state.whetherHoldPanel;// ボタンを押したら状態が変わる
        break;

    /*Climb
     * Climbの流れ
     * 　1.リフトを上げる。
     * 　2.ソレノイドでストッパーを出す。
     *   3.リフトを下げる。(ここで車体が持ち上がる)
     *   4.モーターを回して前に進む
     */

      // 1
      case climbMoveToHab_2Height:
        state.liftSetpoint = Const.hab_2Height;
        state.is_liftPIDOn = true;
        break;

      case climbMoveToHab_3Height:
        state.liftSetpoint = Const.hab_3Height;
        state.is_liftPIDOn = true;
        break;

      // 2
      case climbStopperOn:
        state.is_climbSolenoidOn = true;
        break;

      // 3
      case climbLiftDown:
        state.liftSpeed = -1.0;
        break;

      // 4
      case climbAdvance:
        state.driveXSpeed = 0;
        state.climbMotorSpeed = 1.0;
        break;



    // NoCommand
      case noCommand:
      default:
      state.is_noCommand = false;
    }     
  
    

  if(!state.is_noCommand){
  /*Controller
   * コマンドがなかった時にコントローラーから値を受け取り代入する。
   */
    state.driveXSpeed = deadbandProcessing(joystick.getX());// 右に傾ける(右に回る)と正
    state.driveYSpeed = deadbandProcessing(joystick.getY());

    state.liftSpeed = deadbandProcessing((-joystick.getThrottle()+ 1)/2);// 下で0上で1
    
  }


  /*Substitute
   *  コマンドやコントローラーによって変えられた変数を代入する。
   */
   /*
    drive.applyState(state);
    //lift.applyState(state);
    grabber.applyState(state);

    if(state.is_lineTraceOn){
      //drive.setLineTraceSetpoint();
      //drive.lineTracePIDEnable();
    }else if(state.is_drivePIDOn){
      drive.setSetpoint(state.driveStraightSetpoint, state.driveTurnSetpoint);// PID制御
      drive.PIDEnable();
    }else{
      drive.PIDDisable();
      drive.arcadeDrive(driveXSpeed, driveYSpeed);// 普通のモーター制御
    }


    if(is_liftPIDOn){
      lift.setSetpoint(liftSetpoint);// PID制御
      lift.PIDEnable();
    }else{
      lift.PIDDisable();
      lift.setSpeed(liftSpeed);// 普通のモーター制御
    }

      
    if(whetherHoldCargo){
      grabber.holdCargo();// カーゴを回収
    }else{
      grabber.stopRoller();// ローラーを止める
    }

    if(whetherReleaseCargo){
      grabber.releaseCargo();// カーゴ射出
    }else{
      grabber.stopRoller();// ローラーを止める
    }
      
    if(whetherHoldPanel){
      grabber.holdPanel();// パネルをつかむ
    }else{
      grabber.releasePanel();// パネルを離す
    }
  
    frontClimbSolenoid.set(is_climbSolenoidOn);// 前のベアリング付き爪を出す
    backClimbSolenoid.set(is_climbSolenoidOn);// 後ろのモーター付き爪を出す
    climbMotor.set(climbMotorSpeed);// 後ろのモーターを動かす
    */
  }
}
