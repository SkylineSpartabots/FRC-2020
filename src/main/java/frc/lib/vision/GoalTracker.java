/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.lib.vision;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

import frc.lib.geometry.Pose2d;
import frc.robot.Constants;

/**
 * Add your docs here.
 */
public class GoalTracker {

    public static class TrackReport {
        public Pose2d field_to_target;
        public double latest_timestamp;
        public double stability;
        public int id;

        public TrackReport(GoalTrack track) {
            field_to_target = track.getSmoothedPosition();
            latest_timestamp = track.getLatestTimestamp();
            stability = track.getStability();
            this.id = track.getId();
        }
    }


    public static class TrackReportComparator implements Comparator<TrackReport> {

        double mStabilityWeight;
        double mAgeWeight;
        double mCurrentTimestamp;
        double mSwitchingWeight;
        int mLastTrackId;


        public TrackReportComparator(double stability_weight, double age_weight, double switching_weight, 
                                    int last_track_id, double current_timestamp) {
            mStabilityWeight = stability_weight;
            mAgeWeight = age_weight;
            mSwitchingWeight = switching_weight;
            mLastTrackId = last_track_id;
        }

        double score(TrackReport report) {
            double stability_score = mStabilityWeight * report.stability;
            double age_score = mAgeWeight * 
                Math.max(0, (Constants.kMaxGoalTrackAge - (mCurrentTimestamp - report.latest_timestamp))
                / Constants.kMaxGoalTrackAge);
            double switching_score = report.id == mLastTrackId ? mSwitchingWeight : 0;
            return stability_score + age_score + switching_score;
        }

        @Override
        public int compare(TrackReport track1, TrackReport track2) {
            double diff = score(track1) - score(track2);

            if(diff < 0) {
                return 1;
            } else if(diff > 0) {
                return -1;
            } else {
                return 0;
            }
        }

    }

    List<GoalTrack> mCurrentTracks = new ArrayList<>();
    int mNextId = 0;

    public GoalTracker() {}

    public synchronized void reset() {
        mCurrentTracks.clear();
    }

    public synchronized void update(double timestamp, List<Pose2d> field_to_goals) {
        // Try to update existing tracks
        for (Pose2d target : field_to_goals) {
            boolean hasUpdatedTrack = false;
            for (GoalTrack track : mCurrentTracks) {
                if (!hasUpdatedTrack) {
                    if (track.tryUpdate(timestamp, target)) {
                        hasUpdatedTrack = true;
                    }
                } else {
                    track.emptyUpdate();
                }
            }
            if (!hasUpdatedTrack) {
                // Add a new track.
                // System.out.println("Created new track");
                mCurrentTracks.add(GoalTrack.makeNewTrack(timestamp, target, mNextId));
                mNextId++;
            }
        }

        mCurrentTracks.removeIf(track -> !track.isAlive());
    }

    public synchronized boolean hasTracks() {  
        return !mCurrentTracks.isEmpty();
    }

    public synchronized List<TrackReport> getTracks() {
        List<TrackReport> rv = new ArrayList<>();
        for(GoalTrack track : mCurrentTracks) {
            rv.add(new TrackReport(track));
        }

        return rv;
    }



}
