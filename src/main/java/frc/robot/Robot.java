// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.Arrays;

import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.SPI;
import com.kauailabs.navx.frc.AHRS;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  AHRS ahrs;
  Joystick stick;

  private final Timer grabber_close_timer = new Timer(); // timer for autonomous sequence

  private static final String backwardAuto = "backward";
  private static final String forwardAuto = "Forward";
  private static final String mobilityAndChargerAuto = "mobilitycharger";
  private static final String forwardChargerAuto = "forwardCharger";

  private String m_autoSelected;
  private String m_autoChargerSelected;

  private final SendableChooser<String> m_AutonChooser = new SendableChooser<>();
  private final SendableChooser<String> m_AutonChargerChooser = new SendableChooser<>();

  private static final String k_AutonYesCharger = "Yes Charger";
  private static final String k_AutonNoCharger = "No Charger";

  // private XboxController driverJoystick;
  // private XboxController operatorJoystick;

  private XboxController driverController;
  private XboxController operatorController;

  private MecanumDrive m_robotDrive;
  private static final NeutralMode B_MODE = NeutralMode.Brake; // Set the talons neutralmode to brake

  private static final double defaultSpeed = 0.9;
  private static final double armExtendedSpeed = 0.3;

  private static final int kFrontLeftChannel = 1; // TODO:: CHANGE THESE FROM LAST YEAR
  private static final int kRearLeftChannel = 2;
  private static final int kRearRightChannel = 3;
  private static final int kFrontRightChannel = 4;
  
  private static final int raiseLowerChannel = 5;
  private static final int extenderChannel = 6;
  private static final int talon3Channel = 7;


  private static final int driverJoystickChannel = 0;
  private static final int operatorJoystickChannel = 1;

  private static final boolean GRIP_OPENING = false;
  private static final boolean GRIP_CLOSING = true;

  //Limit switches/
  /*
   * Limit claw open/close
   * Limit Arm turn
   * Limit arm extend
   * 
   * switch at certain lengths to place game pieces
   */
  DigitalInput gripCloseLimit = new DigitalInput(0);
  DigitalInput gripOpenLimit = new DigitalInput(1);
  DigitalInput tempSwitch3 = new DigitalInput(2);
  DigitalInput tempSwitch4 = new DigitalInput(3);
  DigitalInput tempSwitch5 = new DigitalInput(4);
  DigitalInput tempSwitch6 = new DigitalInput(5);

  RelativeEncoder frontLeftEncoder;
  RelativeEncoder backLeftEncoder;
  RelativeEncoder backRightEncoder;
  RelativeEncoder frontRightEncoder;
  RelativeEncoder liftEncoder;

  WPI_TalonSRX raiseLowerTalon;
  WPI_TalonSRX extenderTalon;
  WPI_TalonSRX grabberTalon;

  Boolean grabberCanOpen = true;
  Boolean grabberCanClose = true;
  static boolean gripMem = false; //  state 


  int autonRaiseCount;
  int autonOpenCount;
  
  int autonStepCount;

  // for grippers
  static boolean gripDirection; // 1 for close, 0 for open
  static boolean gripOpenOK, gripCloseOK;

  static boolean passedGripCloseSensor;
  static boolean passedGripOpenSensor;

  static int gripCloseLimitCounter = 0;
  final int gripCloseLimitCounterMax = 5;


  //Limelight
  private PIDController rotationController = new PIDController(0.035, 0, 0);
  private PIDController distanceController = new PIDController(0.15, 0, 0);
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tv = table.getEntry("tv");
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry ta = table.getEntry("ta");

  private static final double cubeNodeAprilTagHeight = .36;
  private static final double substationAprilTagHeight = .59;

  private static final ArrayList<Integer> cubeNodeAprilTagIds = new ArrayList<>(Arrays.asList(1,2,3,6,7,8));
  private static final ArrayList<Integer> substationAprilTagIds = new ArrayList<>(Arrays.asList(4,5));

  Encoder MagEncoder;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    CameraServer.startAutomaticCapture();

    m_AutonChooser.setDefaultOption("Forward", forwardAuto);
    m_AutonChooser.addOption("Backward", backwardAuto);
    m_AutonChooser.addOption("Mobility + Charger", mobilityAndChargerAuto);
    m_AutonChooser.addOption("Score + Charger", forwardChargerAuto);
    SmartDashboard.putData("Auto Route", m_AutonChooser);

    m_AutonChargerChooser.setDefaultOption("No Charger", k_AutonNoCharger);
    m_AutonChargerChooser.addOption("Yes Charger", k_AutonYesCharger);
    SmartDashboard.putData("Auto Charger Choice", m_AutonChargerChooser);

    driverController = new XboxController(driverJoystickChannel);
    operatorController = new XboxController(operatorJoystickChannel);

    CANSparkMax frontLeft = new CANSparkMax(kFrontLeftChannel, MotorType.kBrushless);
    CANSparkMax rearLeft = new CANSparkMax(kRearLeftChannel, MotorType.kBrushless);
    CANSparkMax rearRight = new CANSparkMax(kRearRightChannel, MotorType.kBrushless);
    CANSparkMax frontRight = new CANSparkMax(kFrontRightChannel, MotorType.kBrushless);

    frontLeftEncoder = frontLeft.getEncoder();
    backLeftEncoder = rearLeft.getEncoder();
    backRightEncoder = rearRight.getEncoder();
    frontRightEncoder = frontRight.getEncoder();
    
    raiseLowerTalon = new WPI_TalonSRX(raiseLowerChannel);// 
    extenderTalon = new WPI_TalonSRX(extenderChannel);// 
    grabberTalon = new WPI_TalonSRX(talon3Channel);// 

    autonRaiseCount = 0;
    gripCloseOK = false;

    // Netrual mode is HARDWARE on Sparkma
    frontLeft.setInverted(false);
    rearLeft.setInverted(false);
    rearRight.setInverted(true);
    frontRight.setInverted(true); 

    raiseLowerTalon.setNeutralMode(B_MODE);
    extenderTalon.setNeutralMode(B_MODE);
    grabberTalon.setNeutralMode(B_MODE);

    m_robotDrive = new MecanumDrive(frontLeft, rearLeft, frontRight, rearRight);
    m_robotDrive.setDeadband(0.2);

    MagEncoder = new Encoder(2,3, false, Encoder.EncodingType.k4X);


    // Nav-X
    try {
			/***********************************************************************
			 * navX-MXP:
			 * - Communication via RoboRIO MXP (SPI, I2C, TTL UART) and USB.            
			 * - See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface.
			 * 
			 * navX-Micro:
			 * - Communication via I2C (RoboRIO MXP or Onboard) and USB.
			 * - See http://navx-micro.kauailabs.com/guidance/selecting-an-interface.
			 * 
			 * Multiple navX-model devices on a single robot are supported.
			 ************************************************************************/
          ahrs = new AHRS(SPI.Port.kMXP);
            // ahrs = new AHRS(SerialPort.Port.kMXP, SerialDataType.kProcessedData, (byte)50);
            // ahrs.enableLogging(true);
          ahrs.calibrate();

        } catch (RuntimeException ex ) {
            DriverStation.reportError("Error instantiating navX MXP:  " + ex.getMessage(), true);
        }
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {

    SmartDashboard.putNumber(   "IMU_Yaw",              ahrs.getYaw());
    SmartDashboard.putNumber(   "IMU_Pitch",            ahrs.getPitch());
    SmartDashboard.putNumber(   "IMU_Roll",             ahrs.getRoll());

    
    /* Display tilt-corrected, Magnetometer-based heading (requires             */
    /* magnetometer calibration to be useful)                                   */
    
    SmartDashboard.putNumber(   "IMU_CompassHeading",   ahrs.getCompassHeading());

            /* Quaternion Data                                                          */
        /* Quaternions are fascinating, and are the most compact representation of  */
        /* orientation data.  All of the Yaw, Pitch and Roll Values can be derived  */
        /* from the Quaternions.  If interested in motion processing, knowledge of  */
        /* Quaternions is highly recommended.                                       */
        SmartDashboard.putNumber(   "QuaternionW",          ahrs.getQuaternionW());
        SmartDashboard.putNumber(   "QuaternionX",          ahrs.getQuaternionX());
        SmartDashboard.putNumber(   "QuaternionY",          ahrs.getQuaternionY());
        SmartDashboard.putNumber(   "QuaternionZ",          ahrs.getQuaternionZ());

        // SmartDashboard.putNumber("frontLeftEncoder Position: ", frontLeftEncoder.getPosition());

        // SmartDashboard.putNumber("liftEncoder",  raiseLowerTalon.getSelectedSensorPosition(0));

  }

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_AutonChooser.getSelected();
    m_autoChargerSelected = m_AutonChargerChooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
    frontLeftEncoder.setPosition(0);
    autonRaiseCount = 0;
    autonOpenCount = 0;
    autonStepCount = 0;
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {

    // if(autonOpenCount < 10){
    //   grabberTalon.set(ControlMode.PercentOutput, .5);
    //   autonOpenCount++;
    // }

    m_autoSelected = forwardAuto;
    switch (m_autoSelected) {
      case forwardAuto:
        forwardAuto();
        break;
      case backwardAuto:
        backwardAuto();
        // Put default auto code here
        break;

      case forwardChargerAuto :
        while(frontLeftEncoder.getPosition() > -25){
          m_robotDrive.driveCartesian(-.5,0,0);
          if(frontLeftEncoder.getPosition() > -10){
            raiseLowerTalon.set(ControlMode.PercentOutput, -.75);
          }else{
            raiseLowerTalon.set(ControlMode.PercentOutput, 0);

          }
        }

        while(frontLeftEncoder.getPosition() < 25){
          m_robotDrive.driveCartesian(.5,0,0);
        }

      break;
      case mobilityAndChargerAuto :
      
      while(frontLeftEncoder.getPosition() > -35 && autonStepCount == 0){
        m_robotDrive.driveCartesian(-.5,0,0);
        if(frontLeftEncoder.getPosition() > -10){
          raiseLowerTalon.set(ControlMode.PercentOutput, -.75);
        }else{
          raiseLowerTalon.set(ControlMode.PercentOutput, 0);

        }
      }

      if(frontLeftEncoder.getPosition() <= -35){
        autonStepCount++;
      }

      while(frontLeftEncoder.getPosition() < -10 && autonStepCount == 1){
        m_robotDrive.driveCartesian(0,.5,0);
      }


      while(frontLeftEncoder.getPosition() < 10 && autonStepCount > 1){
        m_robotDrive.driveCartesian(.5,0,0);
      }

      break;

      // while(frontLeftEncoder.getPosition() < 0){
      //   m_robotDrive.driveCartesian(0,-.5,0);
      // }

      // while(frontLeftEncoder.getPosition() < 25){
      //   m_robotDrive.driveCartesian(.5,0,0);
      // }

      // break;
      default:

      break;

    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    m_robotDrive.setMaxOutput(.9); 
    grabber_close_timer.reset();
    grabber_close_timer.start();


  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

    double deadbandedDriveRightX = MathUtil.applyDeadband(driverController.getRightX(),.1);

    SmartDashboard.putBoolean("can gripper close:", gripCloseOK);
    SmartDashboard.putNumber("gripCloseLimitCounter:", gripCloseLimitCounter);
    SmartDashboard.putBoolean("gripCloseLimit.get():", gripCloseLimit.get());




    if(driverController.getLeftY() < .5 && driverController.getLeftY() > -.5){
      m_robotDrive.driveCartesian(
        checkSlowMode(driverController.getLeftY()), 
        0,  
        checkSlowMode(-deadbandedDriveRightX/2));
    }else{
      m_robotDrive.driveCartesian(
        checkSlowMode(driverController.getLeftY()), 
        0,  
        -deadbandedDriveRightX/2);

    }

    // if(operatorController.getLeftY() < 1){
    //   raiseLowerTalon.set(ControlMode.PercentOutput, getJoystickValue(operatorController.getLeftY()));
    // } else {
    //   raiseLowerTalon.set(ControlMode.PercentOutput, 8);

    // }



    if(operatorController.getLeftY() < -.2){
      if(operatorController.getLeftY() < -.85){
        raiseLowerTalon.set(ControlMode.PercentOutput, -.85);
      } else {
        raiseLowerTalon.set(ControlMode.PercentOutput, operatorController.getLeftY());
      }
    }else if(operatorController.getLeftY() > .2){
      if(operatorController.getLeftY() > .85){
        raiseLowerTalon.set(ControlMode.PercentOutput, .85);
      }else {
        raiseLowerTalon.set(ControlMode.PercentOutput, operatorController.getLeftY());
      }
    } else {
      raiseLowerTalon.set(ControlMode.PercentOutput, 0);
    }

    if(operatorController.getRightY() < -.2){
      if(operatorController.getRightY() < -.85){
        extenderTalon.set(ControlMode.PercentOutput, -.85);
      } else {
        extenderTalon.set(ControlMode.PercentOutput, operatorController.getRightY());
      }
    }else if(operatorController.getRightY() > .2){
      if(operatorController.getRightY() > .85){
        extenderTalon.set(ControlMode.PercentOutput, .85);
      }else {
        extenderTalon.set(ControlMode.PercentOutput, operatorController.getRightY());
      }
    } else {
      extenderTalon.set(ControlMode.PercentOutput, 0);
    }


    //Grabber

    if(operatorController.getRightTriggerAxis() > .15 && 
        operatorController.getLeftTriggerAxis() < .15 &&
        // gripCloseLimit.get()
        gripCloseOK
    ){    //Close
      grabberTalon.set(ControlMode.PercentOutput, -.5);
      canGrabberClose();

      // if(passedGripOpenSensor && !gripOpenLimit.get()){passedGripOpenSensor = false;}
    }else if(operatorController.getRightTriggerAxis() < .15 && 
    operatorController.getLeftTriggerAxis() > .15 
    ){ // Open
      grabberTalon.set(ControlMode.PercentOutput, .5);
      gripCloseOK = true;
      gripCloseLimitCounter = 0;
      // if(!gripOpenLimit.get()){passedGripOpenSensor = true;}
    } else {
      grabberTalon.set(ControlMode.PercentOutput, 0);

    }


  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {

  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}

  /*
    Start Position: Just inside of community
    Step 1: Drive forward Until TAG 1 is X Feet away
   */
  public void autonDriveOutCommunity(){}

  /*
   Start position: In front of cube spot on left or right side
   Step 1: move forward until collision with grid.
   Step 2: turn around until Tags 1, 2, 3 seen/straight on
   Step 3: move forward until Tag 1/2/3 is X feet away
   */
  public void autonPushCubeDriveOutCommunity(){}

  public void autonPlaceConeDriveOutCommunity(){}

  public void targetAprilTag(int seekingTargetId){
    NetworkTableEntry validTargets = table.getEntry("json ");
    JSONParser parser = new JSONParser();
    JSONObject masterObject;
    ArrayList<Integer> displayTagArr = new ArrayList<>();
    try {
      masterObject = (JSONObject) parser.parse(validTargets.getString(""));

      JSONObject resultsObject = (JSONObject) masterObject.get("Results");
      JSONArray tagsArray = (JSONArray) resultsObject.get("Fiducial");
      for(Object tag : tagsArray){
        JSONObject jsonTag = (JSONObject) tag;
        if(!(jsonTag instanceof JSONObject)){continue;}

        int targetID = (int)jsonTag.get("fid");
        if(targetID == seekingTargetId){System.out.println("Found");}
      }
    } catch(Exception e){
      throw new RuntimeException(e);
    }
  }

  public double getJoystickValue(Double stickAxisValue) {
    if(Math.abs(stickAxisValue) < 0.15) return 0;
    else return stickAxisValue;
}

  public boolean delayedgripCloseLimit(DigitalInput limitGrabClose){
    if(!limitGrabClose.get()){
      // wait(50);
    }
    return limitGrabClose.get();

  }

  public Double checkSlowMode(Double speed){
    if(driverController.getLeftBumper()){
      return speed / 2;
    } 
    return speed;
  }
  
  // public void checkGrip(){
  //   /******************************************************/
  //   // Gripper Section
  //   /******************************************************/
  //   // if (gripOpenLimit.get())
  //   //   gripOpenOK = false;

  //   // if (!gripOpenOK && gripDirection && gripOpenLimit.get())
  //   //   gripMem = true;
  //   // // detect falling edge of LS to enable 
  //   // if (gripMem && gripDirection && !gripOpenLimit.get())
  //   // {
  //   //   gripOpenOK = true;
  //   //   gripMem = false;
  //   // }

  //   if (!gripCloseOK && gripDirection == GRIP_CLOSING && gripCloseLimit.get())
  //     gripMem = true;

  //   if (gripCloseLimit.get())
  //     gripCloseOK = false; // After tiemr/pulse thing

  //   if (!gripCloseOK && gripDirection == GRIP_CLOSING && gripCloseLimit.get())
  //     gripMem = true;
  //   // detect falling edge of LS to enable 
  //   if (gripMem && !gripDirection && gripCloseLimit.get())
  //   {
  //     gripCloseOK = true;
  //     gripMem = false;
  //   }

  // }

  public boolean canGrabberClose(){
    // Get sensor input when false
    //When sensor is false gripper can no longer close


    if(!gripCloseLimit.get() && gripCloseOK && gripCloseLimitCounter < 1){
      gripCloseLimitCounter = 1;
    } else if(gripCloseLimitCounter < gripCloseLimitCounterMax && gripCloseLimitCounter >= 1){
      System.out.println(gripCloseLimitCounter);
      gripCloseLimitCounter++;
    } else if(gripCloseLimitCounter == gripCloseLimitCounterMax){
      gripCloseOK = false;
    } 








    return gripCloseOK;
  }


  public void forwardAuto(){
    if(frontLeftEncoder.getPosition() > -25){
      m_robotDrive.driveCartesian(-.5,0,0);
    }
    
    if(frontLeftEncoder.getPosition() > -10){
      raiseLowerTalon.set(ControlMode.PercentOutput, -.75);
    }else{
      raiseLowerTalon.set(ControlMode.PercentOutput, 0);

    }
  }

  public void backwardAuto(){
    if(frontLeftEncoder.getPosition() < 25){
      m_robotDrive.driveCartesian(.75,0,0);
    }
    
    if(frontLeftEncoder.getPosition() < 10){
      raiseLowerTalon.set(ControlMode.PercentOutput, -.75);
    }else{
      raiseLowerTalon.set(ControlMode.PercentOutput, 0);
    }
  }
}
