/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.Arrays;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.controllers.Xbox;
import frc.lib.sensors.ColorSensor;
import frc.lib.sensors.ColorSensor.Colors;
import frc.lib.util.CrashTracker;
import frc.robot.loops.Looper;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.SubsystemManager;


/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  //private static Drive mDrive;
  private static Limelight mLimelight;
  private static SubsystemManager subsystems;
  private NetworkTable limelightTable;
  private Looper enabledLooper, disabledLooper;
  private Xbox mDriveController;
  private ColorSensor colorSensor;


  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
    enabledLooper = new Looper();
    disabledLooper = new Looper(); 
    mLimelight = new Limelight(Constants.kShooterLimelightConstants);
    colorSensor = new ColorSensor();
    //mDrive = Drive.getInstance();

    subsystems = SubsystemManager.getInstance();
    mDriveController = new Xbox(0);
    subsystems.setSubsystems(mLimelight);

    subsystems.registerEnabledLoops(enabledLooper);
    subsystems.registerDisabledLoops(disabledLooper);

    //mDrive.zeroSensors();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("Distance Vision", mLimelight.getDistance());
    SmartDashboard.putNumber("Y Offset", mLimelight.getYOffset());
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to
   * the switch structure below with additional strings. If using the
   * SendableChooser make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
    
  }

  /**
   * This function is called periodically during autonomous.
   */
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

  @Override
  public void teleopInit() {
    try {
      disabledLooper.stop();
      enabledLooper.start();
      //SmartDashboard.putBoolean("Auto", false);

      //needs to be commented out
      //drive.zeroSensors();
      //elevator.zeroSensors();
      
    } catch (Throwable t) {
      CrashTracker.logThrowableCrash(t);
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    try {
      mDriveController.update();
      Colors color = colorSensor.getColor();

      //System.out.println("R: " + colorSensor.getRaw().red + " G: " + colorSensor.getRaw().green + " B: " + colorSensor.getRaw().blue);
    
      switch(color) {
        case BLUE:
          System.out.println("Blue");
          break;
        case RED:
          System.out.println("Red");
          break;
        case GREEN:
          System.out.println("Green");
          break;
        case YELLOW:
          System.out.println("Yellow");
          break;
        case UNKNOWN:
          System.out.println("Unk");
          break;
      }


      //mDrive.setCurvatureDrive(mDriveController.getY(Hand.kLeft), -mDriveController.getX(Hand.kRight), false);
      //powerCellFollow();

      
    } catch (Throwable t) {
      CrashTracker.logThrowableCrash(t);
      throw t;
    }
    
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() { // checkSubsystems();
  }

  @Override
  public void disabledInit() {
    enabledLooper.stop();
    disabledLooper.start();
  }
}

