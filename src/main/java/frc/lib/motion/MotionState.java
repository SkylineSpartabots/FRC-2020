/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.lib.motion;

import static frc.lib.motion.MotionUtil.kEpsilon;
import static frc.lib.util.Util.epsilonEquals;
/**
 * Add your docs here.
 */

public class MotionState {
    protected final double t_;
    protected final double pos_;
    protected final double vel_;
    protected final double acc_;

    public static MotionState kInvalidState = new MotionState(Double.NaN, Double.NaN, Double.NaN, Double.NaN);

    public MotionState(double t, double pos, double vel, double acc) {
        t_ = t;
        pos_ = pos;
        vel_ = vel;
        acc_ = acc;
    }

    public MotionState(MotionState state) {
        this(state.t_, state.pos_, state.vel_, state.acc_);
    }

    public double t() {
        return t_;
    }

    public double pos() {
        return pos_;
    }

    public double vel() {
        return vel_;
    }

    public double vel2() {
        return vel_ * vel_;
    }

    public double acc() {
        return acc_;
    }

    public MotionState extrapolate(double t, double acc) {
        final double dt = t - t_;
        return new MotionState(t, pos_ + vel_ * dt + .5 * acc * dt * dt, vel_ + acc * dt, acc);
    }

    public MotionState extrapolate(double t) {
        return extrapolate(t, acc_);
    }

    /**
     * Find the next time (first time > MotionState.t()) that this MotionState will be at pos. This is an inverse of the
     * extrapolate() method.
     *
     * @param pos The position to query.
     * @return The time when we are next at pos() if we are extrapolating with a positive dt. NaN if we never reach pos.
     */
    public double nextTimeAtPos(double pos) {
        if (epsilonEquals(pos, pos_, kEpsilon)) {
            // Already at pos.
            return t_;
        }
        if (epsilonEquals(acc_, 0.0, kEpsilon)) {
            // Zero acceleration case.
            final double delta_pos = pos - pos_;
            if (!epsilonEquals(vel_, 0.0, kEpsilon) && Math.signum(delta_pos) == Math.signum(vel_)) {
                // Constant velocity heading towards pos.
                return delta_pos / vel_ + t_;
            }
            return Double.NaN;
        }

        // Solve the quadratic formula.
        // ax^2 + bx + c == 0
        // x = dt
        // a = .5 * acc
        // b = vel
        // c = this.pos - pos
        final double disc = vel_ * vel_ - 2.0 * acc_ * (pos_ - pos);
        if (disc < 0.0) {
            // Extrapolating this MotionState never reaches the desired pos.
            return Double.NaN;
        }
        final double sqrt_disc = Math.sqrt(disc);
        final double max_dt = (-vel_ + sqrt_disc) / acc_;
        final double min_dt = (-vel_ - sqrt_disc) / acc_;
        if (min_dt >= 0.0 && (max_dt < 0.0 || min_dt < max_dt)) {
            return t_ + min_dt;
        }
        if (max_dt >= 0.0) {
            return t_ + max_dt;
        }
        // We only reach the desired pos in the past.
        return Double.NaN;
    }

    @Override
    public String toString() {
        return "(t=" + t_ + ", pos=" + pos_ + ", vel=" + vel_ + ", acc=" + acc_ + ")";
    }


    @Override
    public boolean equals(Object other) {
        return (other instanceof MotionState) && equals((MotionState) other, kEpsilon);
    }

        /**
     * Checks if two MotionStates are epsilon-equals (all fields are equal within a specified tolerance).
     */
    public boolean equals(MotionState other, double epsilon) {
        return coincident(other, epsilon) && epsilonEquals(acc_, other.acc_, epsilon);
    }

    /**
     * Checks if two MotionStates are coincident (t, pos, and vel are equal within a nominal tolerance, but acceleration
     * may be different).
     */
    public boolean coincident(MotionState other) {
        return coincident(other, kEpsilon);
    }

    /**
     * Checks if two MotionStates are coincident (t, pos, and vel are equal within a specified tolerance, but
     * acceleration may be different).
     */
    public boolean coincident(MotionState other, double epsilon) {
        return epsilonEquals(t_, other.t_, epsilon) && epsilonEquals(pos_, other.pos_, epsilon)
                && epsilonEquals(vel_, other.vel_, epsilon);
    }

    /**
     * Returns a MotionState that is the mirror image of this one. Pos, vel, and acc are all negated, but time is not.
     */
    public MotionState flipped() {
        return new MotionState(t_, -pos_, -vel_, -acc_);
    }
}
