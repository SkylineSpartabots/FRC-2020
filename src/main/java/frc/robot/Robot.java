/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.Optional;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.controllers.Xbox;
import frc.lib.geometry.Pose2d;
import frc.lib.util.CrashTracker;
import frc.lib.util.DriveSignal;
import frc.lib.util.TelemetryUtil;
import frc.lib.util.Util;
import frc.lib.util.TelemetryUtil.PrintStyle;
import frc.robot.auto.ModeExecutor;
import frc.robot.auto.ModeSelector;
import frc.robot.auto.modes.AutoModeBase;
import frc.robot.loops.Looper;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.RobotStateEstimator;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Spinner;
import frc.robot.subsystems.SubsystemManager;
import frc.robot.subsystems.Limelight.LedMode;


public class Robot extends TimedRobot {

  //Loopers
  private final Looper mEnabledLooper = new Looper();
  private final Looper mDisabledLooper = new Looper();

  //Drive team Controllers
  private final Xbox mDriveController = new Xbox(0);
  private final Xbox mOperatorController = new Xbox(1);


  //Subsystems
  private final SubsystemManager mSubsystemManager = SubsystemManager.getInstance();

  //private final Intake mIntake = Intake.getInstance();
  //private final Hopper mHopper = Hopper.getInstance();
  //private final Spinner mSpinner = Spinner.getInstance();
  //private final Climb mClimb = Climb.getInstance();
  //private final LED mLED = LED.getInstance();
  //private final Shooter mShooter = Shooter.getInstance();
  private final Drive mDrive = Drive.getInstance();

  private final RobotState mRobotState = RobotState.getInstance();
  private Limelight mLimelight;
  private final RobotStateEstimator mRobotStateEstimator = RobotStateEstimator.getInstance();


  private ModeSelector mModeSelector = new ModeSelector();
  private ModeExecutor mAutoModeExecutor;
  private ModeExecutor mTestModeExecutor;



  Robot() {
    CrashTracker.logRobotConstruction();
  }


  @Override
  public void robotInit() {
    try {
      CrashTracker.logRobotInit();

      mLimelight = new Limelight(Constants.kShooterLimelightConstants);

      mSubsystemManager.setSubsystems(
        //mDrive,
        mLimelight,
        mRobotStateEstimator
      );

      mSubsystemManager.registerEnabledLoops(mEnabledLooper);
      mSubsystemManager.registerDisabledLoops(mDisabledLooper);

      //mDrive.setHeading(Rotation2d.identity());
      
    } catch (Throwable t) {
      CrashTracker.logThrowableCrash(t);
      throw t;
    }
  }


  @Override
  public void robotPeriodic() {
    try {
      mSubsystemManager.outputToSmartDashboard();
      mRobotState.outputToSmartDashboard();
    } catch (Throwable t) {
      CrashTracker.logThrowableCrash(t);
      throw t;
    }
    
  }

  @Override
  public void disabledInit() {
    try {
      CrashTracker.logDisabledInit();
      mEnabledLooper.stop();

      if(mAutoModeExecutor != null) {
        mAutoModeExecutor.stop();
      }

      if(mTestModeExecutor != null) {
        mTestModeExecutor.stop();
      }

      

      //Zero sensors accordingly
      //mDrive.zeroSensors();
      mRobotState.reset(Timer.getFPGATimestamp(), new Pose2d());


      mModeSelector.reset();
      mModeSelector.updateModeSelection();
      mAutoModeExecutor = new ModeExecutor();
      mTestModeExecutor = new ModeExecutor();

    } catch (Throwable t) {
      CrashTracker.logThrowableCrash(t);
      throw t;
    }
  }

  @Override
  public void disabledPeriodic() {
    try {

      mModeSelector.updateModeSelection();

      Optional<AutoModeBase> autoMode = mModeSelector.getAutoMode();
      Optional<AutoModeBase> testMode = mModeSelector.getTestMode();

      if(autoMode.isPresent() && autoMode.get() != mAutoModeExecutor.getAutoMode()) {
        System.out.println("Set auto mode to: " + autoMode.get().getClass().toString());
        mAutoModeExecutor.setAutoMode(autoMode.get());
      }

      if(testMode.isPresent() && testMode.get() != mTestModeExecutor.getAutoMode()) {
        System.out.println("Set test mode to: " + testMode.get().getClass().toString());
        mTestModeExecutor.setAutoMode(testMode.get());
      }

    } catch (Throwable t) {
      CrashTracker.logThrowableCrash(t);
      throw t;
    }
  }

  @Override
  public void autonomousInit() {
    try {
      CrashTracker.logAutoInit();
      mDisabledLooper.stop();

      mTestModeExecutor.stop();
      //Zero sensors and robot state accordingly

      //mDrive.zeroSensors();
      mRobotState.reset(Timer.getFPGATimestamp(), new Pose2d());
      
      mAutoModeExecutor.start();

      mEnabledLooper.start();


    } catch (Throwable t) {
      CrashTracker.logThrowableCrash(t);
      throw t;
    }
    
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    try {

    } catch (Throwable t) {
      CrashTracker.logThrowableCrash(t);
      throw t;
    }
  }

  @Override
  public void teleopInit() {
    try {
      mDisabledLooper.stop();
      if(mTestModeExecutor != null) {
        mTestModeExecutor.stop();
      }

      if(mAutoModeExecutor != null) {
        mAutoModeExecutor.stop();
      }
      mLimelight.setLed(LedMode.OFF);
      //mDrive.zeroSensors();
      mRobotState.reset(Timer.getFPGATimestamp(), new Pose2d());

      TelemetryUtil.print("Robot Pose X: " + mRobotState.getLatestFieldToVehicle().getValue().getTranslation().x(), 
        PrintStyle.ERROR, false);
      mEnabledLooper.start();
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

    
      SmartDashboard.putString("Target Distance", (int)mLimelight.getDistance()/12 + "', " + mLimelight.getDistance()%12);
      SmartDashboard.putNumber("Y Offset", mLimelight.getYOffset());
      SmartDashboard.putNumber("X Offset", mLimelight.getXOffset());
      //SmartDashboard.putNumber("Heading", mDrive.getHeading().getDegrees());      
    } catch (Throwable t) {
      CrashTracker.logThrowableCrash(t);
      throw t;
    }
    
  }

  @Override
  public void testInit() {
    try {
      CrashTracker.logTestInit();
      mDisabledLooper.stop();
      
      
      mAutoModeExecutor.stop();
      mTestModeExecutor.start();

      mEnabledLooper.start();
      
      
    } catch(Throwable t) {
      CrashTracker.logThrowableCrash(t);
    }
  }

  @Override
  public void testPeriodic() { 
    try {

      
    } catch(Throwable t) {
      CrashTracker.logThrowableCrash(t);
    }
  }

  boolean visionTrackEnabled = false;

  public void driverControl() {

    //mDrive.setArcadeDrive(mDriveController.getY(Hand.kLeft), mDriveController.getX(Hand.kRight));
      
    /*if(mDriveController.bButton.isBeingPressed()) {
      visionTrackEnabled = false;
      mDrive.setTurnPIDTarget(Rotation2d.fromDegrees(20));
    }

    if(mDriveController.aButton.isBeingPressed()) {
      visionTrackEnabled = true;
    }

    if(visionTrackEnabled && mLimelight.seesTarget()) {
      mDrive.setTurnPIDTarget(Rotation2d.fromDegrees(mDrive.getHeading().getDegrees() + mLimelight.getXOffset()));
    }*/

  
  }

 
}

