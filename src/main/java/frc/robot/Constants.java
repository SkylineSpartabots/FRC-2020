/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * Add your docs here.
 */
public class Constants {

	
	public static final int kTimeOutMs = 100;
	public static final double kPathFollowingMaxAccel = 0;
	public static final double kMaxGoalTrackAge = 0;
	public static final double kCameraFrameRate = 0;
	public static final double kMaxTrackerDistance = 0;
	public static final double kLooperDt = 0.01;
	public static final int kDriveWheelTrackWidthInches = 0;
	public static final int kTrackScrubFactor = 0;
	public static final double kMaxLookAheadSpeed = 0;
	public static final double kMinLookAhead = 0;
	public static final double kMaxLookAhead = 0;
	public static final double kMinLookAheadSpeed = 0;
	public static final double kInertiaSteeringGain = 0;
	public static final double kPathFollowingProfileKp = 0;
	public static final double kPathFollowingProfileKffv = 0;
	public static final double kPathFollowingProfileKv = 0;
	public static final double kPathFollowingProfileKs = 0;
	public static final double kPathFollowingMaxVel = 0;
	public static final double kPathFollowingProfileKffa = 0;
	public static final double kPathFollowingGoalVelTolerance = 0;
	public static final double kPathStopSteeringDistance = 0;
	public static final double kPathFollowingGoalPosTolerance = 0;
	public static double kMaxGoalSmoothingTime;
	public static final double kWheelDiameter = 6.0;
	public static double driveVelocityKd;
	public static double kPathFollowingProfileKi;

	public static final double kFalconPPR = 2048;
	public static final double kGearReduction = 1.0;
	public static final double kTicksPerInch = (kFalconPPR) / (kWheelDiameter * Math.PI);

	public static final double kFalconHeatThreshold = 75;
	public static final double kImageCaptureLatency = 0;

}
