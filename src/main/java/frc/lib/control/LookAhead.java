/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.lib.control;

/**
 * Add your docs here.
 */
public class LookAhead {
    public final double min_distance_;
    public final double max_distance_;
    public final double min_speed_;
    public final double max_speed_;

    protected final double delta_distance_;
    protected final double delta_speed_;

    public LookAhead(double min_distance, double max_distance, double min_speed, double max_speed) {
        min_distance_ = min_distance;
        max_distance_ = max_distance;
        min_speed_ = min_speed;
        max_speed_ = max_speed;
        delta_distance_ = max_distance - min_distance;
        delta_speed_ = max_speed - min_speed;
    }

    public double getLookaheadForSpeed(double speed) {
        double lookahead = delta_distance_ * (speed - min_speed_) / delta_speed_  + max_speed_;
        return Double.isNaN(lookahead) ? min_distance_ : Math.max(min_distance_, Math.min(max_distance_, lookahead));
    }
}
