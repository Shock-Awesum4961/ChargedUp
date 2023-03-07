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

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
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

  private static final String backwardAuto = "backward";
  private static final String forwardAuto = "Forward";

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

  //Limit switches/
  /*
   * Limit claw open/close
   * Limit Arm turn
   * Limit arm extend
   * 
   * switch at certain lengths to place game pieces
   */
  DigitalInput tempSwitch1 = new DigitalInput(0);
  DigitalInput tempSwitch2 = new DigitalInput(1);
  DigitalInput tempSwitch3 = new DigitalInput(2);
  DigitalInput tempSwitch4 = new DigitalInput(3);
  DigitalInput tempSwitch5 = new DigitalInput(4);
  DigitalInput tempSwitch6 = new DigitalInput(5);

  RelativeEncoder frontLeftEncoder;
  RelativeEncoder backLeftEncoder;
  RelativeEncoder backRightEncoder;
  RelativeEncoder frontRightEncoder;

  WPI_TalonSRX raiseLowerTalon;
  WPI_TalonSRX extenderTalon;
  WPI_TalonSRX grabberTalon;
  


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

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_AutonChooser.setDefaultOption("Backward", backwardAuto);
    m_AutonChooser.addOption("Forward", forwardAuto);
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

    frontLeft.setInverted(false);
    rearLeft.setInverted(false);
    rearRight.setInverted(true);
    frontRight.setInverted(true); 

    // Netrual mode is HARDWARE on Sparkma
    raiseLowerTalon.setNeutralMode(B_MODE);
    extenderTalon.setNeutralMode(B_MODE);
    grabberTalon.setNeutralMode(B_MODE);
    // rearLeft.setNeutralMode(B_MODE);
    // frontRight.setNeutralMode(B_MODE);
    // rearRight.setNeutralMode(B_MODE);
    // flyWheelMotor.setNeutralMode(C_MODE);

    m_robotDrive = new MecanumDrive(frontLeft, rearLeft, frontRight, rearRight);
    m_robotDrive.setDeadband(0.2);


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

        SmartDashboard.putNumber("frontLeftEncoder Position: ", frontLeftEncoder.getPosition());
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
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    // switch (m_autoSelected) {
    //   case forwardAuto:
    //     while(frontLeftEncoder.getPosition() > -50){
    //       m_robotDrive.driveCartesian(-.5,0,0);

    //     }
    //     // Put custom auto code here
    //     break;
    //   case backwardAuto:
    //   default:
    //     while(frontLeftEncoder.getPosition() < 50){
    //       m_robotDrive.driveCartesian(.5,0,0);

    //     }
    //     // Put default auto code here
    //     break;
    // }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    m_robotDrive.setMaxOutput(.9); 

  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    if(driverController.getLeftY() < .1 && driverController.getLeftY() > -.1){
      m_robotDrive.driveCartesian(driverController.getLeftY(), -driverController.getLeftX()/2,  -driverController.getRightX()/2);
    }else{
      m_robotDrive.driveCartesian(driverController.getLeftY(), -driverController.getLeftX(),  -driverController.getRightX()/2);

    }

    // if(operatorController.getLeftY() < 1){
    //   raiseLowerTalon.set(ControlMode.PercentOutput, getJoystickValue(operatorController.getLeftY()));
    // } else {
    //   raiseLowerTalon.set(ControlMode.PercentOutput, 8);

    // }



    if(operatorController.getLeftY() < -.2){
      if(operatorController.getLeftY() < -.75){
        raiseLowerTalon.set(ControlMode.PercentOutput, -.75);
      } else {
        raiseLowerTalon.set(ControlMode.PercentOutput, operatorController.getLeftY());
      }
    }else if(operatorController.getLeftY() > .2){
      if(operatorController.getLeftY() > .75){
        raiseLowerTalon.set(ControlMode.PercentOutput, .75);
      }else {
        raiseLowerTalon.set(ControlMode.PercentOutput, operatorController.getLeftY());
      }
    } else {
      raiseLowerTalon.set(ControlMode.PercentOutput, 0);
    }

    // if(operatorController.getRightX() < -.2){
    //   if(operatorController.getRightX() < -.75){
    //     grabberTalon.set(ControlMode.PercentOutput, -.75);
    //   } else {
    //     grabberTalon.set(ControlMode.PercentOutput, operatorController.getRightX());
    //   }
    // }else if(operatorController.getRightX() > .2){
    //   if(operatorController.getRightX() > .75){
    //     grabberTalon.set(ControlMode.PercentOutput, .75);
    //   }else {
    //     grabberTalon.set(ControlMode.PercentOutput, operatorController.getRightX());
    //   }
    // } else {
    //   grabberTalon.set(ControlMode.PercentOutput, 0);
    // }
    if(operatorController.getRightX() < -.2){
      if(operatorController.getRightX() < -.75){
        extenderTalon.set(ControlMode.PercentOutput, -.75);
      } else {
        extenderTalon.set(ControlMode.PercentOutput, operatorController.getRightX());
      }
    }else if(operatorController.getRightX() > .2){
      if(operatorController.getRightX() > .75){
        extenderTalon.set(ControlMode.PercentOutput, .75);
      }else {
        extenderTalon.set(ControlMode.PercentOutput, operatorController.getRightX());
      }
    } else {
      extenderTalon.set(ControlMode.PercentOutput, 0);
    }

    if(operatorController.getRightTriggerAxis() > .15 && operatorController.getLeftTriggerAxis() < .15){
      grabberTalon.set(ControlMode.PercentOutput, .5);
    }
    if(operatorController.getRightTriggerAxis() < .15 && operatorController.getLeftTriggerAxis() > .15){
      grabberTalon.set(ControlMode.PercentOutput, -.5);
    }


  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

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
}
