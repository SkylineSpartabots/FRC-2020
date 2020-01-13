/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import frc.lib.geometry.Rotation2d;
import frc.lib.geometry.Twist2d;
import frc.robot.Kinematics;
import frc.robot.RobotState;
import frc.robot.loops.ILooper;
import frc.robot.loops.Loop;


public class RobotStateEstimator extends Subsystem {

    private static RobotStateEstimator mInstance;

    public static RobotStateEstimator getInstance() {
        if(mInstance == null) {
            mInstance = new RobotStateEstimator();
        } 
        return mInstance;
    }

    private Drive mDrive = Drive.getInstance();
    private RobotState mRobotState = RobotState.getInstance();
    private double mLeftPrevDistance = 0.0;
    private double mRightPrevDistance = 0.0;
    private double mPrevTimestamp = -1.0;
    private Rotation2d mPrevHeading = null;

    private RobotStateEstimator() {}

    @Override
    public void registerEnabledLoops(ILooper looper) {
        looper.register(new EnabledLoop());
    }

    private class EnabledLoop implements Loop {

        @Override
        public void onStart(double timestamp) {
            mLeftPrevDistance = mDrive.getLeftEncoderPosition();
            mRightPrevDistance = mDrive.getRightEncoderPosition();
            mPrevTimestamp = timestamp;
        }

        @Override
        public void onLoop(double timestamp) {
            if(mPrevHeading == null) {
                mPrevHeading = mRobotState.getLatestFieldToVehicle().getValue().getRotation();
            }

            final double dt = timestamp - mPrevTimestamp;
            final double leftDisance = mDrive.getLeftEncoderPosition();
            final double rightDistance = mDrive.getRightEncoderPosition();
            final double deltaLeft = leftDisance - mLeftPrevDistance;
            final double deltaRight = rightDistance - mRightPrevDistance;
            final Rotation2d heading = mDrive.getHeading();
            final Twist2d odometry_velocity = mRobotState.generateOdometryFromSensors(deltaLeft, 
                deltaRight, heading);
            final Twist2d predicted_velocity = Kinematics.forwardKinematics(mDrive.getLeftLinearVelocity(),
                mDrive.getRightLinearVelocity());
            //mRobotState.addObservations(timestamp, odometry_velocity, predicted_velocity);
            mLeftPrevDistance = leftDisance;
            mRightPrevDistance = rightDistance;

        }

        @Override
        public void onStop(double timestamp) {

        }

    }


    @Override
    public void stop() {

    }

    @Override
    public boolean checkSystem() {
        return false;
    }

    @Override
    public void outputTelemetry() {

    }
}
