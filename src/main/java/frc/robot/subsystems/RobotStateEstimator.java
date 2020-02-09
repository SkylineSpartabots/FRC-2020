/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import frc.lib.geometry.Pose2d;
import frc.lib.geometry.Rotation2d;
import frc.lib.geometry.Twist2d;
import frc.lib.util.TelemetryUtil;
import frc.lib.util.TelemetryUtil.PrintStyle;
import frc.robot.Kinematics;
import frc.robot.RobotState;
import frc.robot.loops.ILooper;
import frc.robot.loops.Loop;


public class RobotStateEstimator extends Subsystem {

    static RobotStateEstimator mInstance = new RobotStateEstimator();
    private RobotState mRobotState = RobotState.getInstance();
    private Drive mDrive = Drive.getInstance();
    private double left_encoder_prev_distance_ = 0.0;
    private double right_encoder_prev_distance_ = 0.0;
    private double prev_timestamp_ = -1.0;
    private Rotation2d prev_heading_ = null;

    public static RobotStateEstimator getInstance() {
        if (mInstance == null) {
            mInstance = new RobotStateEstimator();
        }

        return mInstance;
    }

    private RobotStateEstimator() {}

    @Override
    public void registerEnabledLoops(ILooper looper) {
        looper.register(new EnabledLoop());
    }

    private class EnabledLoop implements Loop {
        @Override
        public synchronized void onStart(double timestamp) {
            left_encoder_prev_distance_ = mDrive.getLeftEncoderPosition();
            right_encoder_prev_distance_ = mDrive.getRightEncoderPosition();
            prev_timestamp_ = timestamp;
        }

        @Override
        public synchronized void onLoop(double timestamp) {
            if (prev_heading_ == null) {
                prev_heading_ = mRobotState.getLatestFieldToVehicle().getValue().getRotation();
            }
            final double dt = timestamp - prev_timestamp_;
            final double left_distance = mDrive.getLeftEncoderPosition();
            final double right_distance = mDrive.getRightEncoderPosition();
            final double delta_left = left_distance - left_encoder_prev_distance_;
            final double delta_right = right_distance - right_encoder_prev_distance_;
            final Rotation2d gyro_angle = mDrive.getHeading();
            Twist2d odometry_twist;
            synchronized (mRobotState) {
                final Pose2d last_measurement = mRobotState.getLatestFieldToVehicle().getValue();
                odometry_twist = Kinematics.forwardKinematics(last_measurement.getRotation(), delta_left,
                        delta_right, gyro_angle);
            }
            final Twist2d measured_velocity = Kinematics.forwardKinematics(
                    delta_left, delta_right, prev_heading_.inverse().rotateBy(gyro_angle).getRadians()).scaled(1.0 / dt);
            final Twist2d predicted_velocity = Kinematics.forwardKinematics(mDrive.getLeftLinearVelocity(),
                    mDrive.getRightLinearVelocity()).scaled(dt);

            mRobotState.addObservations(timestamp, odometry_twist, measured_velocity,
                    predicted_velocity);
            left_encoder_prev_distance_ = left_distance;
            right_encoder_prev_distance_ = right_distance;
            prev_heading_ = gyro_angle;
            prev_timestamp_ = timestamp;
        }

        @Override
        public void onStop(double timestamp) {}
    }

    @Override
    public void stop() {}

    @Override
    public boolean checkSystem() {
        return true;
    }

    @Override
    public void outputTelemetry() {
        mRobotState.outputToSmartDashboard();
    }
}
