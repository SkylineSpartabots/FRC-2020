/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import frc.lib.geometry.Rotation2d;
import frc.robot.subsystems.Limelight.LimelightConstants;

/**
 * Add your docs here.
 */
public class Constants {

	//Timeout for CAN commands and error checking
	public static final int kTimeOutMs = 100;

	//Cycle speed for looper threads
	public static final double kLooperDt = 0.01;

	//Drive physical characteristics
	public static final double kDriveWheelTrackWidthInches = 26.125;
	public static final double kTrackScrubFactor = 1.0469;
	public static final double kDriveWheelDiameter = 6.0;
	public static final double kDriveWheelRadiusInches = kDriveWheelDiameter / 2.0;
	public static final double kDriveWheelTrackRadiusMeters = kDriveWheelTrackWidthInches / 2.0 * 0.0254;


	//Drive
	public static final double kMinLookAhead = 12.0;
	public static final double kMinLookAheadSpeed = 12.0;
	public static final double kMaxLookAhead = 48.0;
	public static final double kMaxLookAheadSpeed = 120.0;
	
	public static final double kPathFollowingMaxAccel = 80.0;
	public static final double kPathFollowingMaxVel = 120.0;
	public static final double kPathFollowingProfileKp = 0.3 / 12.0;
	public static final double kPathFollowingProfileKi = 0.0;
	public static final double kPathFollowingProfileKv = 0.01 / 12.0;
	public static final double kPathFollowingProfileKffv = 0.003889;
	public static final double kPathFollowingProfileKffa = 0.001415;
	public static final double kPathFollowingProfileKs = 0.1801 / 12.0;
	public static final double kPathFollowingGoalPosTolerance = 3.0;
	public static final double kPathFollowingGoalVelTolerance = 12.0;
	public static final double kPathStopSteeringDistance = 12.0;
	
	public static final double kInertiaSteeringGain = 0.0;

	public static final double kMaxGoalTrackAge = 0;
	public static final double kCameraFrameRate = 0;
	public static final double kMaxTrackerDistance = 0;
	
	

	
	
	
	
	
	
	

	public static double kMaxGoalSmoothingTime;
	public static double driveVelocityKd;
	

	public static final double kFalconPPR = 2048;
	public static final double kGearReduction = 1.0;
	public static final double kTicksPerInch = (kFalconPPR) / (kDriveWheelDiameter * Math.PI);

	public static final double kFalconHeatThreshold = 75;
	
	public static final double kFlywheelDiameter = 4.0;
	public static final double kNeoPPR = 42;
	public static final double kShooterGearReduction = 0.5;
	public static final double kShooterRampRate = 0;
	public static final int kShooterKfBufferSize = 20;

	public static final double kHorizontalFOV = 59.6; // degrees
    public static final double kVerticalFOV = 49.7; // degrees
    public static final double kVPW = 2.0 * Math.tan(Math.toRadians(kHorizontalFOV / 2.0));
    public static final double kVPH = 2.0 * Math.tan(Math.toRadians(kVerticalFOV / 2.0));
	public static final double kImageCaptureLatency = 11.0 / 1000.0;;



	public static final double kLensHeight = 27.06;
	public static final double kLensHorizontalAngle = 15;
	public static final double kTargetHeight = 98.5;


	public static final LimelightConstants kShooterLimelightConstants = new LimelightConstants();
	public static final double kQuickStopThreshold = 0;
	/* The threshold to begin accumulating quickStopAccumulator */ 
	public static final double kQuickStopAlpha = 0.05;
	public static final double kShooterkP = 0.09;
	public static final double kShooterkI = 0.00008;
	public static final double kShooterkD = 0;
	public static final double kShooterkF = 0.03;
	public static final double kShooterIZone = 0;
	public static final double kShooterStartOnTargetRpm = 0;
	public static final double kShooterStopOnTargetRpm = 0;
	public static final int kShooterMinOnTargetSamples = 0;
	

	static {
		kShooterLimelightConstants.kName = "Shooter Limelight";
		kShooterLimelightConstants.kHeight = 12;
		kShooterLimelightConstants.kTableName = "limelight";
		kShooterLimelightConstants.kHorizontalToLens = Rotation2d.fromDegrees(38.0);
	}

}
