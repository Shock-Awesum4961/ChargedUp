// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.Arrays;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.SPI;
import com.kauailabs.navx.frc.AHRS;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  AHRS ahrs;
  Joystick stick;
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private String m_autoChargerSelected;
  private final SendableChooser<String> m_AutonChooser = new SendableChooser<>();
  private final SendableChooser<String> m_AutonChargerChooser = new SendableChooser<>();

  private static final String k_AutonYesCharger = "Yes Charger";
  private static final String k_AutonNoCharger = "No Charger";


  private Joystick driverJoystick;

  private MecanumDrive m_robotDrive;
  private static final NeutralMode B_MODE = NeutralMode.Brake; // Set the talons neutralmode to brake

  private static final int kFrontLeftChannel = 12; // TODO:: CHANGE THESE FROM LAST YEAR
  private static final int kRearLeftChannel = 14;
  private static final int kFrontRightChannel = 22;
  private static final int kRearRightChannel = 20;
  private static final int driverJoystickChannel = 0;


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
    m_AutonChooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_AutonChooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto Route", m_AutonChooser);

    m_AutonChargerChooser.setDefaultOption("No Charger", k_AutonNoCharger);
    m_AutonChargerChooser.addOption("Yes Charger", k_AutonYesCharger);
    SmartDashboard.putData("Auto Charger Choice", m_AutonChargerChooser);

    driverJoystick = new Joystick(driverJoystickChannel);

    WPI_TalonSRX frontLeft = new WPI_TalonSRX(kFrontLeftChannel);//  
    WPI_TalonSRX rearLeft = new WPI_TalonSRX(kRearLeftChannel);// 
    WPI_TalonSRX frontRight = new WPI_TalonSRX(kFrontRightChannel);// 
    WPI_TalonSRX rearRight = new WPI_TalonSRX(kRearRightChannel);// 

    frontRight.setInverted(true); 
    rearRight.setInverted(true);
    frontLeft.setInverted(false);
    rearLeft.setInverted(false);

    frontLeft.setNeutralMode(B_MODE);
    rearLeft.setNeutralMode(B_MODE);
    frontRight.setNeutralMode(B_MODE);
    rearRight.setNeutralMode(B_MODE);
    // flyWheelMotor.setNeutralMode(C_MODE);


    m_robotDrive = new MecanumDrive(frontLeft, rearLeft, frontRight, rearRight);

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
            //ahrs = new AHRS(SerialPort.Port.kMXP, SerialDataType.kProcessedData, (byte)50);
            ahrs.enableLogging(true);
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
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    m_robotDrive.driveCartesian(driverJoystick.getY(), -driverJoystick.getX(),  -driverJoystick.getRawAxis(4));

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
}
