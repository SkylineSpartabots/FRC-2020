/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.auto;

import java.util.Optional;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.util.TelemetryUtil;
import frc.lib.util.TelemetryUtil.PrintStyle;
import frc.robot.auto.modes.AutoModeBase;
import frc.robot.auto.modes.DoNothing;
import frc.robot.auto.modes.tests.DriveConfiguration;

public class ModeSelector {

    private enum StartingPosition {
        NEAR_TARGET, MID_TARGET, FAR_TARGET
    }

    private enum AutoModes {
        DO_NOTHING,
        LEAVE_LINE,
        TRENCH_AUTO
    }

    private enum TestModes {
        FULL,
        DRIVE,
        DRIVE_CONFIGURATION,
        DRIVE_CHARACTERIZATION
    }



    private AutoModes mDesiredAutoMode = null;
    private StartingPosition mStartingPosition = null;
    private TestModes mDesiredTestMode = null;

    private SendableChooser<AutoModes> mAutoModeChooser;
    private SendableChooser<StartingPosition> mStartPositionChooser;
    private SendableChooser<TestModes> mTestModeChooser;

    private Optional<AutoModeBase> mAutoMode = Optional.empty();
    private Optional<AutoModeBase> mTestMode = Optional.empty();


    public ModeSelector() {
        //Auto start position sendable chooser configuration
        mStartPositionChooser = new SendableChooser<>();
        mStartPositionChooser.setDefaultOption("Closest to Target", StartingPosition.NEAR_TARGET);
        mStartPositionChooser.addOption("Middle Position", StartingPosition.MID_TARGET);
        mStartPositionChooser.addOption("Furthest from Target", StartingPosition.FAR_TARGET);

        SmartDashboard.putData("Auto Starting Position", mStartPositionChooser);

        //Auto mode sendable chooser configuration
        mAutoModeChooser = new SendableChooser<>();
        mAutoModeChooser.setDefaultOption("Full Trench", AutoModes.TRENCH_AUTO);
        mAutoModeChooser.addOption("Do Nothing", AutoModes.DO_NOTHING);
        mAutoModeChooser.addOption("Leave Line", AutoModes.LEAVE_LINE);

        SmartDashboard.putData("Auto Mode", mAutoModeChooser);

        //Test mode sendable chooser configuration
        mTestModeChooser = new SendableChooser<>();
        mTestModeChooser.setDefaultOption("All Systems Check", TestModes.FULL);
        mTestModeChooser.addOption("Drive Configuration", TestModes.DRIVE_CONFIGURATION);
        mTestModeChooser.addOption("Drive Characterization", TestModes.DRIVE_CHARACTERIZATION);
        mTestModeChooser.addOption("Drive", TestModes.DRIVE);

        SmartDashboard.putData("Test Mode", mTestModeChooser);
    }


    public void updateModeSelection() {
        StartingPosition desiredStart = mStartPositionChooser.getSelected();
        AutoModes desiredAuto = mAutoModeChooser.getSelected();
        TestModes desiredTest = mTestModeChooser.getSelected();

        if(mStartingPosition != desiredStart || mDesiredAutoMode != desiredAuto 
            || mDesiredTestMode != desiredTest) {
            
            TelemetryUtil.print("Updating choosers: Starting Position -> " + desiredStart.name()
                + ", Auto Mode -> " + desiredAuto.name() + ", Test Mode -> " + desiredTest.name(),
                PrintStyle.INFO, true);
            mAutoMode = getAutoModeForParams(desiredAuto, desiredStart);
            mTestMode = getTestModeForParams(desiredTest);
        }

        mStartingPosition = desiredStart;
        mDesiredAutoMode = desiredAuto;
        mDesiredTestMode = desiredTest;
    }


    private Optional<AutoModeBase> getAutoModeForParams(AutoModes mode, StartingPosition position) {
        switch(mode) {
            case DO_NOTHING:
                return Optional.of(new DoNothing());
            //add more cases for auto modes to come
            default:
                break;
        }

        TelemetryUtil.print("No valid auto mode found for " + mode.name(), PrintStyle.ERROR, true);
        return Optional.empty();
    }

    private Optional<AutoModeBase> getTestModeForParams(TestModes mode) {
        switch(mode) {
            case FULL:
                //return 
            case DRIVE_CONFIGURATION:
                return Optional.of(new DriveConfiguration());
            case DRIVE_CHARACTERIZATION:
                //return
            case DRIVE:
                //return
        }

        TelemetryUtil.print("No valid test mode found for " + mode.name(), PrintStyle.ERROR, true);
        return Optional.empty();
    }


    public void reset() {
        mAutoMode = Optional.empty();
        mTestMode = Optional.empty();
        mDesiredAutoMode = null;
        mDesiredTestMode = null;
        mStartingPosition = null;
    }


    public Optional<AutoModeBase> getAutoMode() {
        return mAutoMode;
    }

    public Optional<AutoModeBase> getTestMode() {
        return mTestMode;
    }
}
