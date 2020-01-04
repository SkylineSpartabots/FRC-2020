/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.lib.motion;

/**
 * Constraints for constructing a MotionProfile.
 */
public class MotionProfileConstraints {
    protected double max_abs_vel = Double.POSITIVE_INFINITY;
    protected double max_abs_acc = Double.POSITIVE_INFINITY;

    public MotionProfileConstraints(double max_vel, double max_acc) {
        this.max_abs_vel = Math.abs(max_vel);
        this.max_abs_acc = Math.abs(max_acc);
    }

    /**
     * @return The (positive) maximum allowed velocity.
     */
    public double max_abs_vel() {
        return max_abs_vel;
    }

    /**
     * @return The (positive) maximum allowed acceleration.
     */
    public double max_abs_acc() {
        return max_abs_acc;
    }

    @Override
    public boolean equals(Object obj) {
        if (!(obj instanceof MotionProfileConstraints)) {
            return false;
        }
        final MotionProfileConstraints other = (MotionProfileConstraints) obj;
        return (other.max_abs_acc() == max_abs_acc()) && (other.max_abs_vel() == max_abs_vel());
    }
}
