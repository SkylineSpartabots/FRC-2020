/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;


public class Constants {

	// Timeout for CAN commands and error checking
	public static final int kTimeOutMs = 10;

	// Cycle speed for looper threads
	public static final double kLooperDt = 0.01;

	// Drive
	public static final double kDriveWheelTrackWidthMeters = 23.0 * 0.0254; //26.125
	public static final double kDriveWheelDiameter = 6.0;
	public static final double kDriveWheelDiameterInMeters = kDriveWheelDiameter * 0.0254;
	public static final double kDriveWheelRadiusInches = kDriveWheelDiameter / 2.0;
	public static final double kDriveWheelTrackRadiusMeters = kDriveWheelTrackWidthMeters / 2.0;
	
	public static final double kQuickStopThreshold = 0;
	public static final double kQuickStopAlpha = 0.05;

	public static final double kDriveKaVolts = 0;
	public static final double kDriveKvVolts = 0;
	public static final double kDriveKsVolts = 0;
	public static final double kDriveMaxVelocity = 0;
	public static final double kDriveMaxAcceleration = 0;

	// Shooter	
	public static final double kShooterRampRate = 0;
	public static final int kShooterKfBufferSize = 20;
	public static final double kShooterkP = 0.09;
	public static final double kShooterkI = 0.00008;
	public static final double kShooterkD = 0;
	public static final double kShooterkF = 0.03;
	public static final int kShooterIZone = 0;
	public static final double kShooterStartOnTargetRpm = 0;
	public static final double kShooterStopOnTargetRpm = 0;
	public static final int kShooterMinOnTargetSamples = 0;

	// Limelight
	public static final double kLensHeight = 27.06;
	public static final double kLensHorizontalAngle = 15;
	public static final double kTargetHeight = 98.5;

	
	//Spinner
	public static final int kControlPanelProximityThreshold = 0;
	public static final int kNeoPPR = 42;
	public static final double kSpinnerWheelDiameterInches = 2.0;
	public static final int kControlPanelDiameterInches = 32;
	public static final double kSpinnerCountsPerInch = kNeoPPR / (Math.PI * kSpinnerWheelDiameterInches);
	public static final double kCountsPerControlPanelRotation = kControlPanelDiameterInches * kSpinnerCountsPerInch;
	
	//Climb
	public static final double kSlideDownToWinchTransitionTime = 0.1;
	public static final double kSlideDownEncoderTarget = 100;
	public static final double kHookSlideWaitHeightThreshold = 900;
	
	// Air Compressor
	public static final double kCompressorShutOffCurrent = 0;

	public static final double kRotationControlPercentOutput = 0;

	

	

}
