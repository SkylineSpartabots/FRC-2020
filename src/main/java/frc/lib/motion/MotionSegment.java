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
public class MotionSegment {

    protected MotionState start_;
    protected MotionState end_;

    public MotionSegment(MotionState start, MotionState end) {
        start_ = start;
        end_ = end;

    }

    /**
     * Verifies that:
     * <p>
     * 1. All segments have a constant acceleration.
     * <p>
     * 2. All segments have monotonic position (sign of velocity doesn't change).
     * <p>
     * 3. The time, position, velocity, and acceleration of the profile are consistent.
     */
    public boolean isValid() {
        if (!epsilonEquals(start().acc(), end().acc(), kEpsilon)) {
            // Acceleration is not constant within the segment.
            System.err.println(
                    "Segment acceleration not constant! Start acc: " + start().acc() + ", End acc: " + end().acc());
            return false;
        }
        if (Math.signum(start().vel()) * Math.signum(end().vel()) < 0.0 && !epsilonEquals(start().vel(), 0.0, kEpsilon)
                && !epsilonEquals(end().vel(), 0.0, kEpsilon)) {
            // Velocity direction reverses within the segment.
            System.err.println("Segment velocity reverses! Start vel: " + start().vel() + ", End vel: " + end().vel());
            return false;
        }
        if (!start().extrapolate(end().t()).equals(end())) {
            // A single segment is not consistent.
            if (start().t() == end().t() && Double.isInfinite(start().acc())) {
                // One allowed exception: If acc is infinite and dt is zero.
                return true;
            }
            System.err.println("Segment not consistent! Start: " + start() + ", End: " + end());
            return false;
        }
        return true;
    }

    public boolean containsTime(double t) {
        return t >= start().t() && t <= end().t();
    }

    public boolean containsPos(double pos) {
        return pos >= start().pos() && pos <= end().pos() || pos <= start().pos() && pos >= end().pos();
    }

    public MotionState start() {
        return start_;
    }

    public void setStart(MotionState start) {
        start_ = start;
    }

    public MotionState end() {
        return end_;
    }

    public void setEnd(MotionState end) {
        end_ = end;
    }

    @Override
    public String toString() {
        return "Start: " + start() + ", End: " + end();
    }
}
