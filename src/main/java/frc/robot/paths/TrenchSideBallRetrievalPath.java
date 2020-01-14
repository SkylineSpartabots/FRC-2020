/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.paths;

import java.util.ArrayList;

import frc.lib.control.Path;
import frc.robot.paths.PathBuilder.Waypoint;

/**
 * Add your docs here.
 */
public class TrenchSideBallRetrievalPath implements PathContainer {

    public static final String kStartIntakingMarker = "START_INTAKING_MARKER";

    boolean mIsBlue;

    public TrenchSideBallRetrievalPath(boolean isBlue) {
        mIsBlue = isBlue;
    }

    @Override
    public Path buildPath() {
        ArrayList<Waypoint> waypoints = new ArrayList<Waypoint>();
        waypoints.add(new Waypoint(0.0, 0.0, 0.0, 0.0));
        waypoints.add(new Waypoint(121.04, 60.52, 0.0, 120.0, kStartIntakingMarker));

        return PathBuilder.buildPathFromWaypoints(waypoints);
    }

    @Override
    public boolean isReversed() {
        return true;
    }
}
