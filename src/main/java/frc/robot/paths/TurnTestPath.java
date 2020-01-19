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
 * Test path for turning
 */
public class TurnTestPath implements PathContainer {

    @Override
    public Path buildPath() {
        ArrayList<Waypoint> waypoints = new ArrayList<>();
        waypoints.add(new Waypoint(0.0, 0.0, 0.0, 0.0));

        waypoints.add(new Waypoint(0.0, 0.0, 45.0, 0.0));
        waypoints.add(new Waypoint(0.0, 0.0, 0.0, 0.0));
        waypoints.add(new Waypoint(0.0, 0.0, 90.0, 0.0));
        waypoints.add(new Waypoint(0.0, 0.0, 0.0, 0.0));

        waypoints.add(new Waypoint(0.0, 0.0, -45.0, 0.0));
        waypoints.add(new Waypoint(0.0, 0.0, 0.0, 0.0));
        waypoints.add(new Waypoint(0.0, 0.0, -90.0, 0.0));
        waypoints.add(new Waypoint(0.0, 0.0, 0.0, 0.0));

        waypoints.add(new Waypoint(0.0, 0.0, 180, 0.0));
        waypoints.add(new Waypoint(0.0, 0.0, 0.0, 0.0));
        waypoints.add(new Waypoint(0.0, 0.0, 360, 0.0));
        waypoints.add(new Waypoint(0.0, 0.0, 0.0, 0.0));
        return PathBuilder.buildPathFromWaypoints(waypoints);
    }

    @Override
    public boolean isReversed() {
        return false;
    }
}
