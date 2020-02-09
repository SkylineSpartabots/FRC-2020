package frc.robot;

import java.util.Map;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.geometry.Pose2d;
import frc.lib.geometry.Rotation2d;
import frc.lib.geometry.Twist2d;
import frc.lib.util.InterpolatingDouble;
import frc.lib.util.InterpolatingTreeMap;
import frc.lib.util.MovingAverageTwist2d;
import frc.robot.subsystems.Drive;

public class RobotState {
    private static RobotState mInstance;

    public static RobotState getInstance() {
        if (mInstance == null) {
            mInstance = new RobotState();
        }
        return mInstance;
    }

    private static final int kObservationBufferSize = 100;

    private InterpolatingTreeMap<InterpolatingDouble, Pose2d> field_to_vehicle_;
    private Twist2d vehicle_velocity_predicted_;
    private Twist2d vehicle_velocity_measured_;
    private MovingAverageTwist2d vehicle_velocity_measured_filtered_;
    private double distance_driven_;

    private RobotState() {
        reset(0.0, Pose2d.identity());
    }

    /**
     * Resets the field to robot transform (robot's position on the field)
     */
    public synchronized void reset(double start_time, Pose2d initial_field_to_vehicle) {
        field_to_vehicle_ = new InterpolatingTreeMap<>(kObservationBufferSize);
        field_to_vehicle_.put(new InterpolatingDouble(start_time), initial_field_to_vehicle);
        vehicle_velocity_predicted_ = Twist2d.identity();
        vehicle_velocity_measured_ = Twist2d.identity();
        vehicle_velocity_measured_filtered_ = new MovingAverageTwist2d(25);
        distance_driven_ = 0.0;
    }

    public synchronized void reset() {
        reset(Timer.getFPGATimestamp(), Pose2d.identity());
    }

    /**
     * Returns the robot's position on the field at a certain time. Linearly interpolates between stored robot positions
     * to fill in the gaps.
     */
    public synchronized Pose2d getFieldToVehicle(double timestamp) {
        return field_to_vehicle_.getInterpolated(new InterpolatingDouble(timestamp));
    }


    public synchronized Map.Entry<InterpolatingDouble, Pose2d> getLatestFieldToVehicle() {
        return field_to_vehicle_.lastEntry();
    }

    public synchronized Pose2d getPredictedFieldToVehicle(double lookahead_time) {
        return getLatestFieldToVehicle().getValue()
                .transformBy(Pose2d.exp(vehicle_velocity_predicted_.scaled(lookahead_time)));
    }

    public synchronized void addFieldToVehicleObservation(double timestamp, Pose2d observation) {
        field_to_vehicle_.put(new InterpolatingDouble(timestamp), observation);
    }

    public synchronized void addObservations(double timestamp, Twist2d displacement, Twist2d measured_velocity,
                                             Twist2d predicted_velocity) {
        distance_driven_ += displacement.dx;
        addFieldToVehicleObservation(timestamp,
                Kinematics.integrateForwardKinematics(getLatestFieldToVehicle().getValue(), displacement));
        vehicle_velocity_measured_ = measured_velocity;
        if (Math.abs(vehicle_velocity_measured_.dtheta) < 2.0 * Math.PI) {
            // Reject really high angular velocities from the filter.
            vehicle_velocity_measured_filtered_.add(vehicle_velocity_measured_);
        } else {
            vehicle_velocity_measured_filtered_.add(new Twist2d(vehicle_velocity_measured_.dx, vehicle_velocity_measured_.dy, 0.0));
        }
        vehicle_velocity_predicted_ = predicted_velocity;
    }

    public synchronized double getDistanceDriven() {
        return distance_driven_;
    }

    public synchronized void resetDistanceDriven() {
        distance_driven_ = 0.0;
    }

    public synchronized Twist2d getPredictedVelocity() {
        return vehicle_velocity_predicted_;
    }

    public synchronized Twist2d getMeasuredVelocity() {
        return vehicle_velocity_measured_;
    }

    public synchronized Twist2d getSmoothedVelocity() {
        return vehicle_velocity_measured_filtered_.getAverage();
    }


    public void outputToSmartDashboard() {
        SmartDashboard.putNumber("Latest X Position", getLatestFieldToVehicle().getValue().getTranslation().x());
        SmartDashboard.putNumber("Latest Y Position", getLatestFieldToVehicle().getValue().getTranslation().y());

        //SmartDashboard.putNumber("Drive Left Velocity", Drive.getInstance().getLeftLinearVelocity());
        //SmartDashboard.putNumber("Drive Right Velocity", Drive.getInstance().getRightLinearVelocity());

        SmartDashboard.putNumber("Distance Driven", distance_driven_);

        SmartDashboard.putNumber("Odometry Heading", getLatestFieldToVehicle().getValue().getRotation().getDegrees());
    }
    

}