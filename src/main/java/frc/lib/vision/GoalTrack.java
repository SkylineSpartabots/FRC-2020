    /*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.lib.vision;

import java.util.Map;
import java.util.TreeMap;

import edu.wpi.first.wpilibj.Timer;
import frc.lib.geometry.Pose2d;
import frc.lib.geometry.Rotation2d;
import frc.robot.Constants;

/**
 * Add your docs here.
 */
public class GoalTrack {
    TreeMap<Double, Pose2d> mObservedPositions = new TreeMap<>();
    Pose2d mSmoothedPosition = null;
    int mId;

    private GoalTrack() {}

    public static synchronized GoalTrack makeNewTrack(double timestamp, Pose2d first_observation, int id) {
        GoalTrack rv = new GoalTrack();
        rv.mObservedPositions.put(timestamp, first_observation);
        rv.mSmoothedPosition = first_observation;
        rv.mId = id;
        return rv;
    }

    public synchronized boolean isAlive() {
        return mObservedPositions.size() > 0;
    }

    public synchronized void emptyUpdate() {
        pruneByTime();
    }

    public synchronized boolean tryUpdate(double timestamp, Pose2d new_observation) {
        if(!isAlive()) {
            return false;
        }

        double distance = mSmoothedPosition.inverse().transformBy(new_observation).getTranslation().norm();
        if(distance < Constants.kMaxTrackerDistance) {
            mObservedPositions.put(timestamp, new_observation);
            pruneByTime();
            return true;
        } else {
            emptyUpdate();
            return false;
        }
    }

    synchronized void smooth() {
        if(isAlive()) {
            double x = 0;
            double y = 0;
            double s = 0;
            double c = 0;
            double t_now = Timer.getFPGATimestamp();
            int num_samples = 0;

            for(Map.Entry<Double, Pose2d> entry : mObservedPositions.entrySet()) {
                if(t_now - entry.getKey() > Constants.kMaxGoalSmoothingTime) {
                    continue;
                }
                num_samples++;
                x += entry.getValue().getTranslation().x();
                y += entry.getValue().getTranslation().y();
                c += entry.getValue().getRotation().cos();
                s += entry.getValue().getRotation().sin();
            }

            x /= num_samples;
            y /= num_samples;
            s /= num_samples;
            c /= num_samples;

            if(num_samples == 0) {
                mSmoothedPosition = mObservedPositions.lastEntry().getValue();
            } else {
                mSmoothedPosition = new Pose2d(x, y, new Rotation2d(c, s, true));
            }

        }
    }

    synchronized void pruneByTime() {
        double delete_before = Timer.getFPGATimestamp() - Constants.kMaxGoalTrackAge;
        mObservedPositions.entrySet().removeIf(entry -> entry.getKey() < delete_before);
        if(mObservedPositions.isEmpty()) {
            mSmoothedPosition = null;
        } else {
            smooth();
        }
    }

    public synchronized Pose2d getSmoothedPosition() {
        return mSmoothedPosition;
    }

    public synchronized Pose2d getLatestPosition() {
        return mObservedPositions.lastEntry().getValue();
    }

    public synchronized double getLatestTimestamp() {
        return mObservedPositions.keySet().stream().max(Double::compareTo).orElse(0.0);
    }

    public synchronized double getStability() {
        return Math.min(1.0, mObservedPositions.size() / (Constants.kCameraFrameRate * Constants.kMaxGoalTrackAge));
    }

    public synchronized int getId() {
        return mId;
    }

    
}
