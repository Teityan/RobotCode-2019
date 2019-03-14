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
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.VictorSP;

public class Robot extends TimedRobot {
    private State state;

    // Controller
    private XboxController driver, operator;

    // Motors
    private Spark driveLeft, driveRight;
    private Talon liftMotor;
    private VictorSP rollerMotor;
    private VictorSP climbMotor;

    // Encoder, Gyro
    private Encoder rightDriveEncoder, leftDriveEncoder;
    private EncoderGroup driveEncoder;
    private Encoder liftEncoder;
    private ADXRS450_Gyro gyro;

    // Solenoid
    private Solenoid armSolenoid, barSolenoid;
    private Solenoid climbSolenoid;

    // Compressor
    private Compressor compressor;

    // Timer for climb
    private Timer climbTimer;

    // SubModule
    private Drive drive;
    private Lift lift;
    private Grabber grabber;
    private Climb climb;

    // ライントレース用のセンサー　
    // 0～3.3V 白線があると電圧が上がる 
    //private AnalogInput rightFrontSensor, 
    //                  rightBackSensor, 
    //                  leftFrontSensor, 
    //                  leftBackSensor;

    // Camera
    private CameraServer elevatorCamera, frameCamera;

    // NetWorkTable
    private NetworkTable networkTable;


    // variables
    // ToDo
    //private double distanceToLine[] = new double[2], 
    //                displayPosition[] = new double[2];

    // functions
    private double deadbandProcessing(double value) {
        return Math.abs(value) > Const.Deadband ? value : 0 ;
    }

    public double[] geLinePosition(double[] displayPosition) {
        double theta_c = Const.Theta_Camera_rad;    // カメラ自体の角度
        double theta_a = Const.Theta_Angle_rad;    // カメラの画角
        double maxH = Const.cameraHeight;    //カメラの設置された高さ


        // xは中央から右向き正  yは下から上向き正
        double x = displayPosition[0];
        double y = displayPosition[1];
        double result[] = new double[2]; 

        double distanceCamera_Display = Math.sqrt(y*y + maxH/Math.cos(theta_c)*maxH/Math.cos(theta_c) - 2*y*maxH*Math.sin(theta_a)/Math.cos(theta_c));    //余弦定理

        double maxY = maxH / Math.cos(theta_c) * Math.sin(theta_a) *2;    // 縦幅
        double maxX = 2 * distanceCamera_Display * Math.tan(theta_a);    // 横幅

        // double aboveMaxX = maxH * Math.tan(theta_a);
        // double minH = maxH - maxY * Math.cos(Math.PI/2 - (theta_c + theta_a));     
    

        // y方向の距離を求める
        double sinDistanceCamera_Display = y * Math.cos(theta_a) / distanceCamera_Display;    // 正弦定理
        double angleDisplay_LineOfSight  = Math.PI/2 - theta_a + Math.asin(sinDistanceCamera_Display);    // 目線の一番低いところを0とした時の角度
        double l = y * Math.sin(angleDisplay_LineOfSight) / Math.sin(angleDisplay_LineOfSight + theta_c + theta_a);    // 目線の一番低いところからの距離
        result[1] = maxH * Math.tan(theta_c) + l;

        // x方向の距離を求める
        double distanceCamera_DisplayZ = distanceCamera_Display * Math.cos(theta_c + angleDisplay_LineOfSight);    //yの位置からdisplayとカメラとの距離を求める
        result[0]  = maxH * x / distanceCamera_DisplayZ; 

        return result;
    }  
  
    @Override
    public void robotInit() {
        // Controller
        //joystick = new Joystick(Const.JoystickPort);
        driver = new XboxController(Const.DriveControllerPort);
        operator = new XboxController(Const.OperateControllerPort);

        // Motors
        driveRight = new Spark(Const.DriveRightPort);
        driveLeft = new Spark(Const.DriveLeftPort);

        liftMotor = new Talon(Const.LiftMotorPort);

        rollerMotor = new VictorSP(Const.RollerMotorPort);

        climbMotor = new VictorSP(Const.ClimbMotorPort);

        rightDriveEncoder = new Encoder(Const.RightDriveEncoderAPort, Const.RightDriveEncoderBPort);
        leftDriveEncoder = new Encoder(Const.LeftDriveEncoderAPort, Const.LeftDriveEncoderBPort);
        driveEncoder = new EncoderGroup(rightDriveEncoder, leftDriveEncoder);    // エンコーダをまとめる   
        driveEncoder.setDistancePerPulse(Const.DriveEncoderDistancePerPulse);

		liftEncoder = new Encoder(Const.LiftEncoderAPort, Const.LiftEncoderBPort);
		liftEncoder.setDistancePerPulse(Const.LiftEncoderDistancePerPulse);

        gyro = new ADXRS450_Gyro();

        // Solenoid
        armSolenoid = new Solenoid(Const.ArmSolenoidPort);
        barSolenoid = new Solenoid(Const.BarSolenoidPort);

        climbSolenoid = new Solenoid(Const.ClimbSolenoidPort);

        // Compressor
        compressor = new Compressor();

        // Sensor
        //rightFrontSensor = new AnalogInput(Const.RightFrontSensorPort);
        //rightBackSensor = new AnalogInput(Const.RightBackSensorPort);
        //leftFrontSensor = new AnalogInput(Const.LeftFrontSensorPort);
        //leftBackSensor = new AnalogInput(Const.LeftBackSensorPort);
        //

        // Climb Timer
        climbTimer = new Timer();

        // Camera
        elevatorCamera = CameraServer.getInstance();
        elevatorCamera.startAutomaticCapture(0);

        frameCamera = CameraServer.getInstance();
        frameCamera.startAutomaticCapture(1);
        
        // NetworkTable
        //networkTable = networkTable.getSubTable(Const.LineFindNetworkTable);

        // State
        state = new State();

        // Submodules
        drive = new Drive(driveLeft, driveRight, driveEncoder, gyro);
        lift = new Lift(liftMotor, liftEncoder);
        grabber = new Grabber(rollerMotor, barSolenoid, armSolenoid);
        climb = new Climb(climbMotor, climbSolenoid);
    }
  
    @Override
    public void autonomousInit() {
    }

    @Override
    public void autonomousPeriodic() {
        teleopPeriodic();
    }

    @Override
    public void teleopInit() {
    }
  
    @Override
    public void teleopPeriodic() {
        /*Init
         * これからコマンドやコントローラーで変数の値を変えてから代入する。
         * 操作しないときは出力を出したくないため、最初に出力を出さない状態で初期化する。
         */
        state.stateInit();

        /*
         * Joystickやセンサーからの入力を受け取ってstate objectに値を流しこむ
         */
		/********** Drive ***********/
		/*
        if (driver.getBumper(Hand.kLeft)) {
            state.driveState = State.DriveState.kLineTrace;
        } else if (driver.getBumper(Hand.kRight)) {
            state.driveState = State.DriveState.kCloseToLine;
        } else */{
            state.driveState = State.DriveState.kManual;
            state.driveStraightSpeed = deadbandProcessing(driver.getY(Hand.kLeft));
            state.driveRotateSpeed = deadbandProcessing(driver.getX(Hand.kRight));

            state.climbMotorSpeed = deadbandProcessing(driver.getY(Hand.kRight));
        }

        if(driver.getBumper(Hand.kLeft)) {
            // 左のバンパーのボタンが押されたら低出力モード
            state.is_lowInputOn = true;
        }

        /********** Lift ***********/
        /**
         * Liftは全てoperatorが操作する
         */
        if(operator.getTriggerAxis(Hand.kLeft) > Const.Deadband){
            // LeftTriggerでカーゴを打ち出す
            state.liftSetpoint = Const.LaunchCargoHeight;
            state.is_liftPIDOn = true;
        }else if (operator.getYButton()) {
            // YでRocketの二段目のCargo
            state.liftSetpoint = Const.RocketSecondCargoHeight;
            state.is_liftPIDOn = true;
            state.is_lowInputOn = true;
        } else if (operator.getAButton()) {
            // AでRocketの一段目のCargo
            state.liftSetpoint = Const.RocketFirstCargoHeight;
            state.is_liftPIDOn = true;
            state.is_lowInputOn = true;
        } else if (operator.getBButton()) {
            // BでRocketCargo ShipのCargo
            state.liftSetpoint = Const.ShipCargoHeight;
            state.is_liftPIDOn = true;
            state.is_lowInputOn = true;
        } else if (operator.getXButton()) {
            // XでRocketのの２段目のHatch
            state.liftSetpoint = Const.RocketSecondHatchHeight;
            state.is_liftPIDOn = true;
            state.is_lowInputOn = true;
        }else {
            // それ以外の場合は手動操作
            state.liftSpeed = deadbandProcessing(-operator.getY(Hand.kLeft));
            state.is_liftPIDOn = false;
        }

        /*********** Grabber ***********/
        /**
         * Cargoを掴む部分
         * driverが操作する
         */
        if (driver.getTriggerAxis(Hand.kRight) > Const.Deadband) {
            // Right Triggerで掴む
            state.cargoState = State.CargoState.kHold;
        } else if (driver.getTriggerAxis(Hand.kLeft) > Const.Deadband) {
            // Left Triggerで離す
            state.cargoState = State.CargoState.kRelease;
        } else {
            // 普段はなにもしない
            state.cargoState = State.CargoState.kDoNothing;
        }

        if (driver.getBumper(Hand.kRight)) {
            // RightBumperボタンでパネルを掴む（掴むところが広がる）
            state.is_toHoldPanel = true;
        } else {
            // 普段は縮まっている
            state.is_toHoldPanel = false;
        }

        if (operator.getBumper(Hand.kLeft)) {
            // Right BumberでArmをしまう
            state.is_toRetractArm = true;
        } else {
            state.is_toRetractArm = false;
        }

		/**
		 * Climb
         * 
         * クライムの流れ...
         * StartButton : HABの上に乗るまでの処理
         *    kDoNothing : kLiftUpと同じ。初期化した後はここからkLiftUpに行く。
         *    kLiftUp    : リフトを枠の、止めるところの少し上へ上げる。
         *    kLocking   : ストッパーを出す。ここでリフトが下がらないように維持する。
         *            　   出す命令をしてから即座にリフトを下げると止めるところに引っかからない可能性があるので0.3秒置く
         *   kAdvance    : リフトを下げて枠にひっかけてロボットを持ち上げる。
         *                 ロボットを持ち上げるのには-1.0くらいの入力が必要だが、最初からそれを入れるとアームに負担がかかるので手動でゆっくり入力する。
         *                 DriveBaseと枠についたモーターを動かして前に進み、HABに乗る。
         *            　
         * 
         * BackButton : HABに乗っかった後の処理。本体が上に乗っかった後も枠がストッパーで下に押し付けられてるのでそれを外す。ストッパーがロックされてたら実行。
         *   kDoNothing  : kLiftUpと同じ。初期化した後はここからkLiftUpに行く。
         *   kLiftUp     : リフトを枠の、止めるところの少し上へ上げる。
         *   kUnlocking  : ストッパーを外す。ここでリフトが下がらないように維持する。
         *            　   外す命令をしてから即座にリフトを下げるとストッパーが外れない可能性があるので0.3秒置く。
         *   kLiftDown   : リフトを自重で下げさせる。枠がHABの手前の壁に引っかかってるかもしれないのでdrvierの右のスティックで枠のモーターを動かす。
         *   
         * 
		 */
        if(operator.getStartButton()) {
			// スタートボタン + A or Yでクライムを始める
            state.is_autoClimbOn = true;
            state.is_lowInputOn = true;
            switch(state.climbSequence) {
				case kDoNothing:
				case kLiftUp:
					if(operator.getTriggerAxis(Hand.kRight) > Const.Deadband) {
						// StartとRightTriggerでHABのLEVEL2までリフトを上げる
                    	state.liftSetpoint = Const.HabSecondHeight;		
						state.is_liftPIDOn = true;
					}else if(operator.getBumper(Hand.kRight)) {
						// StartとRightBumperでHABのLEVEL3までリフトを上げる
						state.liftSetpoint = Const.HabThirdHeight;
						state.is_liftPIDOn = true;
					}else{
						//コマンドがなかったら抜ける
						break;
					}
					
                    if(lift.is_PIDOnTarget()) {
						// 届いたらストッパーを出す
						state.climbSequence = State.ClimbSequence.kLocking;
						// ストッパーを出す時間を考慮して時間計る
                        climbTimer.reset();
						climbTimer.start();
                    }
                    break;

				case kLocking:
					// ストッパーを出す
                    state.is_lockingClimb = true;
					
					// リフトが下がらないように維持する
					state.liftSpeed = Const.KeepLiftHeightSpeed;

                    if(climbTimer.get() > 0.3) {
						// 時間がたったら前に進む
						state.climbSequence = State.ClimbSequence.kAdvance;
						state.is_lockedClimb = true;
                    }
                    break;
                
				case kAdvance:
                    // スティックで前に進む
                    state.driveStraightSpeed = deadbandProcessing(driver.getY(Hand.kLeft));
                    state.climbMotorSpeed = state.driveStraightSpeed;
                    
                    // コンプレッサーを止めてできるだけ負担を減らす。
                    compressor.stop();
					break; 

				default:
					break;
			}

        } else if(operator.getBackButton() && state.is_lockedClimb) {
			// 十分前に進んで後輪がHABに乗ったら実行
			switch(state.climbSequence) {
                case kDoNothing:
				case kLiftUp:
					// 乗っかったら後処理
					// リフトを上げる
					if(operator.getTriggerAxis(Hand.kRight) > Const.Deadband) {
						// StartとRightTriggerでHABのLEVEL2までリフトを上げる
                    	state.liftSetpoint = Const.HabSecondHeight;		
						state.is_liftPIDOn = true;
					}else if(operator.getBumper(Hand.kRight)) {
						// StartとRightBumperでHABのLEVEL3までリフトを上げる
						state.liftSetpoint = Const.HabThirdHeight;
						state.is_liftPIDOn = true;
                    }

					if(lift.is_PIDOnTarget()){
						// 届いたらストッパーを外す
						state.climbSequence = State.ClimbSequence.kUnlocking;
						climbTimer.reset();
						climbTimer.start();
					}	
					break;

				case kUnlocking:
					// ストッパーを外す
					state.is_lockingClimb = false;

					// リフトが下がらないように維持する
					state.liftSpeed = Const.KeepLiftHeightSpeed;

					if(climbTimer.get() > 0.3) {
						// 時間がたったらリフトを下げる
						state.climbSequence = State.ClimbSequence.kLiftDown;
					}
					break;

				case kLiftDown:
					// リフトを下げる
                    state.liftSpeed = 0;
					break;
				
				default: 
					break;
			}

		}else {
			state.is_autoClimbOn = false;
            // 初期化
            state.climbSequence = State.ClimbSequence.kDoNothing;
            compressor.start();
		}

        /*
        　* Stateをapplyする
        　*/
        drive.applyState(state);
        lift.applyState(state);
        grabber.applyState(state);
		climb.applyState(state);

    }

    public void robotPeriidic(){
        drive.printVariables();
        lift.printVariables();
		state.printVariables();
    }
}
