/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.lib.vision;

import frc.lib.geometry.Pose2d;
import frc.lib.geometry.Rotation2d;

public class AimingParameters {
    private final double range;
    private final Pose2d robot_to_goal;
    private final Pose2d field_to_goal;
    private final Rotation2d robot_to_goal_rotation;
    private final double last_seen_timestamp;
    private final double stability;
    private final Rotation2d field_to_vision_target_normal;
    private final int track_id;

    public AimingParameters(Pose2d robot_to_goal, Pose2d field_to_goal, Rotation2d field_to_vision_target_normal,
                                double last_seen_timestamp, double stability, int track_id) {
        this.robot_to_goal = robot_to_goal;
        this.field_to_goal = field_to_goal;
        this.field_to_vision_target_normal = field_to_vision_target_normal;
        this.last_seen_timestamp = last_seen_timestamp;
        this.stability = stability;
        this.track_id = track_id;
        this.range = robot_to_goal.getTranslation().norm();
        this.robot_to_goal_rotation = robot_to_goal.getTranslation().direction();
    }

    public Pose2d getRobotToGoal() {
        return robot_to_goal;
    }

    public Pose2d getFieldToGoal() {
        return field_to_goal;
    }

    public double getRange() {
        return range;
    }

    public Rotation2d getRobotToGoalRotation() {
        return robot_to_goal_rotation;
    }

    public double getLastSeenTimestamp() {
        return last_seen_timestamp;
    }

    public double getStability() {
        return stability;
    }

    public Rotation2d getFieldToVisionTargetNormal() {
        return field_to_vision_target_normal;
    }

    public int getTrackId() {
        return track_id;
    }

}
