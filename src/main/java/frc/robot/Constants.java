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

	public static final double kDriveKaVolts = 0.2;
	public static final double kDriveKvVolts = 1.98;
	public static final double kDriveKsVolts = 0.22;
	public static final double kDriveMaxVelocity = 2; //4.75
	public static final double kDriveMaxAcceleration = 2; //3.166

	// Shooter	
	public static final double kShooterRampRate = 0.2;
	public static final int kShooterKfBufferSize = 20;
	public static final double kShooterkP = 0.00;
	public static final double kShooterkI = 0.00;
	public static final double kShooterkD = 0;
	public static final double kShooterkF = 0.049;
	public static final int kShooterIZone = 0;
	public static final double kShooterStartOnTargetRpm = 300;
	public static final double kShooterStopOnTargetRpm = 200;
	public static final int kShooterMinOnTargetSamples = 20;
	public static final double kRawVelocityToRpm = 2048.0/6000.0;

	// Limelight
	public static final double kLensHeight = 26;
	public static final double kLensHorizontalAngle = 60;
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

	public static final double kStandardShootVelocity = 6500;

	public static final double kDriveAlignControlGainSchedule = 0;

	

	

}
