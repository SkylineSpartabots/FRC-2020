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
public class TestPath1 implements PathContainer {


    boolean mLeft;
    public TestPath1(boolean left) {
        mLeft = left;
    }


    @Override
    public Path buildPath() {
        ArrayList<Waypoint> waypoints = new ArrayList<Waypoint>();
        waypoints.add(new PathBuilder.Waypoint(0, 0.0, 0, 0));
        waypoints.add(new PathBuilder.Waypoint(50, 35, 0, 20));
        waypoints.add(new PathBuilder.Waypoint(100, 70, 0, 20));
        //waypoints.add(new PathBuilder.Waypoint(100.0, 20.0, 90.0, 20.0));
        //waypoints.add(new PathBuilder.Waypoint(80.0, 40.0, 180.0, 20.0));
        //waypoints.add(new PathBuilder.Waypoint(100, 40, 0, 10.0));
        
    //    waypoints.add(new PathBuilder.Waypoint(100, 10, 0, 30.0)); // 30, 0 , 0 , 40
        //waypoints.add(new PathBuilder.Waypoint(110, 30, 10, 40.0));
        //waypoints.add(new PathBuilder.Waypoint(205, 3, 0, 120));


        return PathBuilder.buildPathFromWaypoints(waypoints);
    }

    @Override
    public boolean isReversed() {
        return false;
    }
}
