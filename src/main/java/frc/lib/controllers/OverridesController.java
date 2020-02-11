/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.lib.controllers;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;

public class OverridesController {

    private static OverridesController mInstance = null;
    public static OverridesController getInstance() {
        if(mInstance == null) {
            mInstance = new OverridesController();
        }
        return mInstance;
    }

    private final Joystick mJoystick;

    public Override visionOverride, shooterClosedLoopOverride, navxOverride, driveEncoderOverride,
        spinnerColorSensorOverride, spinnerEncoderOverride;

    private final int VISION = 1; //arduino port: 4
    private final int SHOOTER_CLOSED_LOOP = 2; //arduino port: 5
    private final int NAVX = 3; //arduino port: 3
    private final int DRIVE_ENCODER = 4; //arduino port: 2
    private final int COLOR_SENSOR = 5; //arduino port: 10
    private final int SPINNER_ENCODER = 6; //arduino port: 11


    private OverridesController() {
        mJoystick = new Joystick(2);

        visionOverride = new Override(VISION);
        shooterClosedLoopOverride = new Override(SHOOTER_CLOSED_LOOP);
        navxOverride = new Override(NAVX);
        driveEncoderOverride = new Override(DRIVE_ENCODER);
        spinnerColorSensorOverride = new Override(COLOR_SENSOR);
        spinnerEncoderOverride = new Override(SPINNER_ENCODER);
    }


    public class Override {

        private final int mButtonNumber;
        public Override(int buttonNumber) {
            mButtonNumber = buttonNumber;
        }

        private boolean mPrevIsActive = false;
        private double mStateChangedTimestamp = 0.0;
        private double mStateChangeTimeThreshold = 0.1;

        private boolean mIsEnabled = false;

        public boolean isEnabled() {
            return mIsEnabled;
        }


        public void update() {
            boolean isActive = mJoystick.getRawButton(mButtonNumber);

            if(isActive) {
                if(!mPrevIsActive) {
                    mStateChangedTimestamp = Timer.getFPGATimestamp();
                } else {
                    mIsEnabled = Timer.getFPGATimestamp() - mStateChangedTimestamp > mStateChangeTimeThreshold;
                }
            } else {
                if(mPrevIsActive) {
                    mStateChangedTimestamp = Timer.getFPGATimestamp();
                } else {
                    mIsEnabled = !(Timer.getFPGATimestamp() - mStateChangedTimestamp > mStateChangeTimeThreshold);
                }
            }

            mPrevIsActive = isActive;
        }

    }

    public void update() {
        visionOverride.update();
        shooterClosedLoopOverride.update();
        navxOverride.update();
        driveEncoderOverride.update();
        spinnerColorSensorOverride.update();
        spinnerEncoderOverride.update();
    }

}
