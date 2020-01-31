/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.auto.modes;

import java.util.Arrays;

import frc.robot.auto.ModeEndedException;
import frc.robot.auto.actions.DrivePathAction;
import frc.robot.auto.actions.SeriesAction;
import frc.robot.auto.actions.WaitAction;
import frc.robot.paths.TestPath1;
import frc.robot.paths.TestPath2;

/**
 * Add your docs here.
 */
public class TestPathingMode extends AutoModeBase {

    DrivePathAction mFirstPath;
    DrivePathAction mSecondPath;

    boolean mLeft;

    public TestPathingMode(boolean left) {
        mLeft = left;

        mFirstPath = new DrivePathAction(new TestPath1(left));
        //mSecondPath = new DrivePathAction(new TestPath2(left));
        
    }

    @Override
    protected void routine() throws ModeEndedException {
        runAction((mFirstPath));
    }
}
