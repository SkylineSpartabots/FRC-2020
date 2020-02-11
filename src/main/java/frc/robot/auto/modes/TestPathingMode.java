/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.auto.modes;

import java.util.Arrays;
import java.util.List;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import frc.robot.auto.ModeEndedException;
import frc.robot.auto.actions.DrivePathAction;
import frc.robot.auto.actions.SeriesAction;
import frc.robot.auto.actions.WaitAction;
import frc.robot.paths.PathGenerator;

/**
 * Add your docs here.
 */
public class TestPathingMode extends AutoModeBase {

    DrivePathAction mFirstPath;
    DrivePathAction mSecondPath;

    boolean mLeft;


    public TestPathingMode(boolean left) {
       mLeft = left;
       mFirstPath = new DrivePathAction(paths.startPositionOneToTrenchEnd, true);
       mSecondPath = new DrivePathAction(paths.trenchEndToShootLocation, true);
    }

    @Override
    protected void routine() throws ModeEndedException {
        runAction(mFirstPath);
        runAction(mSecondPath);
    }
}
