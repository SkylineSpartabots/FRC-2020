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
public class TestPath2 implements PathContainer {

    boolean mLeft;
    public TestPath2(boolean left) {
        mLeft = left;
    }


    @Override
    public Path buildPath() {
        ArrayList<Waypoint> waypoints = new ArrayList<Waypoint>();
        waypoints.add(new Waypoint(0, 0.0, 0, 0));
        waypoints.add(new Waypoint(30, 0.0, 0, 40.0));
        //waypoints.add(new Waypoint(110, 1.5, 1, 60.0));
        waypoints.add(new Waypoint(205, 3, 0, 60));
        return PathBuilder.buildPathFromWaypoints(waypoints);
    }

    @Override
    public boolean isReversed() {
        return true;
    }
}
