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
import edu.wpi.first.wpilibj.Watchdog;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.controllers.Xbox;
import frc.lib.util.CrashTracker;
import frc.lib.util.DriveSignal;
import frc.lib.util.TelemetryUtil;
import frc.lib.util.Util;
import frc.lib.util.TelemetryUtil.PrintStyle;
import frc.robot.auto.ModeExecutor;
import frc.robot.auto.ModeSelector;
import frc.robot.auto.modes.AutoModeBase;
import frc.robot.loops.Looper;
import frc.robot.paths.PathGenerator;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Spinner;
import frc.robot.subsystems.SubsystemManager;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Climb.ClimbControlState;
import frc.robot.subsystems.Hopper.HopperControlState;
import frc.robot.subsystems.Intake.IntakeControlState;
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

  private final Intake mIntake = Intake.getInstance();
  private final Hopper mHopper = Hopper.getInstance();
  //private final Spinner mSpinner = Spinner.getInstance();
  //private final Climb mClimb = Climb.getInstance();
  //private final LED mLED = LED.getInstance();
  private final Shooter mShooter = Shooter.getInstance();
  private final Drive mDrive = Drive.getInstance();
  private final Superstructure mSuperstructure = Superstructure.getInstance();

  private Limelight mLimelight = Limelight.getInstance();


  private final PathGenerator mPathGenerator = PathGenerator.getInstance();
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

    
      mSubsystemManager.setSubsystems(
        mDrive,
        mLimelight,
        mIntake,
        mHopper,
        mShooter,
        //mClimb,
        mSuperstructure
      );

      mSubsystemManager.registerEnabledLoops(mEnabledLooper);
      mSubsystemManager.registerDisabledLoops(mDisabledLooper);

      mSubsystemManager.zeroSensors();

      
      mPathGenerator.generatePaths();
      
    } catch (Throwable t) {
      CrashTracker.logThrowableCrash(t);
      throw t;
    }
  }


  @Override
  public void robotPeriodic() {
    try {
      mSubsystemManager.outputToSmartDashboard();
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
      mDisabledLooper.start();

      mLimelight.setLed(LedMode.OFF);

      if(mAutoModeExecutor != null) {
        mAutoModeExecutor.stop();
      }

      if(mTestModeExecutor != null) {
        mTestModeExecutor.stop();
      }


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

      mDrive.setBrakeMode(false);
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

      mDrive.zeroSensors();
      mDrive.resetOdometry(new Pose2d(new Translation2d(0.0, 0.0), mDrive.getHeading()));
      
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
      mLimelight.setLed(LedMode.ON);

      mSubsystemManager.zeroSensors();
      mDrive.resetOdometry(new Pose2d(new Translation2d(0.0, 0.0), mDrive.getHeading()));

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
      mOperatorController.update();

      driverControl();
      
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
      
      
     // mAutoModeExecutor.stop();
      //mTestModeExecutor.start();

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

  



  private boolean deployIntake = true;
  private boolean automatedControlEnabled = false;

  public void driverControl() {


    /* Complete automation controls:
        Start rotation control - Dpad up
        Start position control - Dpad down
        Start auto shoot sequence - B
    */

   if(mOperatorController.dpadUp.isBeingPressed()) {
      if(!automatedControlEnabled) {
        TelemetryUtil.print("Auto", PrintStyle.ERROR, false);
        mSuperstructure.autoShootSequence();
        automatedControlEnabled = true;
      }
    } /*else if(mOperatorController.dpadUp.isBeingPressed()) {
      if(!automatedControlEnabled) {
        mSuperstructure.autoRotationControl();
        automatedControlEnabled = true;
      }
    } else if(mOperatorController.dpadDown.isBeingPressed()) {
      if(!automatedControlEnabled) {
        mSuperstructure.autoPositionControl();
        automatedControlEnabled = true;
      }
    } */else if(mOperatorController.rightBumper.isBeingPressed()) {
      if(!automatedControlEnabled) {
        mDrive.setAlignToTarget();
        automatedControlEnabled = true;
      }
    } else {
      if(automatedControlEnabled) {
        mSuperstructure.clearRequests();
      }
      automatedControlEnabled = false;
    }


    




    /* Drive Controls:
        Curvature Drive: Left joy y, right joy x
        Quick-spin Override: Back
    */


  


    if(!automatedControlEnabled) {
      if(mOperatorController.xButton.isBeingPressed()) {
        mDrive.setAlignToTarget();
      } else {
        mDrive.setArcadeDrive(mDriveController.getY(Hand.kLeft), mDriveController.getX(Hand.kRight));
      }
    }
    
    
    

   /* Intake Controls:
        Intake On - Left Bumper
        Retract Intake - Y
    */

    if(!automatedControlEnabled) {
      if(mOperatorController.yButton.wasActivated()) {
        deployIntake = false;
      }
  
      if(mOperatorController.leftBumper.isBeingPressed()) {
        mIntake.conformToState(IntakeControlState.INTAKE);
        deployIntake = true;
      } else {
        if(deployIntake) {
          mIntake.conformToState(IntakeControlState.STORE);
        } else {
          mIntake.conformToState(IntakeControlState.OFF);
        }
      }
    }

   
    /* Shooter controls:
        Start shooter ramp - Left paddle
        Start and continue auto shooter sequence - B
        Auto-align to shooter target - right bumper
        Open loop shooter control - Right trigger
        Feign Ball - x (optional)
    */

    if(!automatedControlEnabled) {
      if(mOperatorController.rightTrigger.isBeingPressed()) {
        mShooter.setOpenLoop(1.0);
      } else if(mOperatorController.bButton.isBeingPressed()) {
        mShooter.setHoldWhenReady(8700);
      } else if(!automatedControlEnabled) {
        mShooter.setOpenLoop(0.0);
      }
    }
    
    

    /* Spinner controls:
        Start rotation control - Dpad up
        Start position control - Dpad down
        Retract spinner - Left d pad
        Open loop on spinner - right joy x
    */





    /* Hopper Control:
        Open loop hopper control - left trigger
    */

    if(!automatedControlEnabled) {
      if(mOperatorController.leftTrigger.isBeingPressed()) {
        mHopper.conformToState(HopperControlState.INDEX);
      } else {
        mHopper.conformToState(HopperControlState.OFF);
      }
    }




    /* Climb Control:
        Deploy climb: back
        Retract climb: start
    */
    /*if(mOperatorController.backButton.isBeingPressed()) {
      mClimb.conformToState(ClimbControlState.RAISE_HOOK);
    } else if(mOperatorController.startButton.isBeingPressed()) {
      mClimb.conformToState(ClimbControlState.AUTO_WINCH_UP);
    } else {
      mClimb.conformToState(ClimbControlState.OFF);
    }*/


    
    
  
  }

 
}

