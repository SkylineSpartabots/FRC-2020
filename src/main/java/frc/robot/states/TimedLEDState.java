/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.states;

/**
 * Add your docs here.
 */
public interface TimedLEDState {
    void getCurrentLEDState(LEDState desiredState, double timestamp);

    class FlashingLEDState implements TimedLEDState {

        public static FlashingLEDState kBootFault = new FlashingLEDState(LEDState.kRed, 0.75);
        public static FlashingLEDState kRuntimeFault = new FlashingLEDState(LEDState.kRed, 0.25);

        public static FlashingLEDState kShooterAligning = new FlashingLEDState(LEDState.kGreen, 0.25);
        public static FlashingLEDState kTargetNotVisible = new FlashingLEDState(LEDState.kBlue, 0.25);

        public static FlashingLEDState kClimbNotBalanced = new FlashingLEDState(LEDState.kWhite, LEDState.kRed, 0.4);
        public static FlashingLEDState kClimbIsBalanced = new FlashingLEDState(LEDState.kWhite, LEDState.kGreen, 0.4);

        public static FlashingLEDState kIntakingWithVision = new FlashingLEDState(LEDState.kYellow, LEDState.kGreen, 0.25);
        public static FlashingLEDState kIntakingWithoutVision = new FlashingLEDState(LEDState.kYellow, 0.25);

        public static FlashingLEDState kPathFollowing = new FlashingLEDState(LEDState.kOrange, 0.25);
        public static FlashingLEDState kTestMode = new FlashingLEDState(LEDState.kOrange, LEDState.kGreen, 0.25);
        public static FlashingLEDState kManualOverrideEngaged = new FlashingLEDState(LEDState.kPurple, 0.25);


        LEDState mStateOne = new LEDState(0.0, 0.0, 0.0);
        LEDState mStateTwo = new LEDState(0.0, 0.0, 0.0);
        double mDuration;

        public FlashingLEDState(LEDState stateOne, LEDState stateTwo, double duration) {
            mStateOne = stateOne;
            mStateTwo = stateTwo;
            mDuration = duration;
        }

        public FlashingLEDState(LEDState state, double duration) {
            this(LEDState.kOff, state, duration);
        }

        @Override
        public void getCurrentLEDState(LEDState desiredState, double timestamp) {
            if((int) (timestamp / mDuration) % 2 == 0) {
                desiredState.copyFrom(mStateOne);
            } else {
                desiredState.copyFrom(mStateTwo);
            }

        }

    }

    class StaticLEDState implements TimedLEDState {

        public static StaticLEDState kEmergency = new StaticLEDState(LEDState.kRed);
        public static StaticLEDState kShooting = new StaticLEDState(LEDState.kGreen);
        public static StaticLEDState kAutoComplete = new StaticLEDState(LEDState.kOrange);
        public static StaticLEDState kHopperFull = new StaticLEDState(LEDState.kYellow);
        public static StaticLEDState kDisabled = new StaticLEDState(LEDState.kOff);
        public static StaticLEDState kEnabled = new StaticLEDState(LEDState.kWhite);

        LEDState mStaticState = new LEDState(0.0, 0.0, 0.0);

        public StaticLEDState(LEDState staticState) {
            mStaticState.copyFrom(staticState);
        }

        @Override
        public void getCurrentLEDState(LEDState desiredState, double timestamp) {
            desiredState.copyFrom(mStaticState);
        }

    }


}
