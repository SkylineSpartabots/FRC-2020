/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.auto.actions;

import frc.lib.control.Path;
import frc.lib.util.DriveSignal;
import frc.robot.paths.PathContainer;
import frc.robot.subsystems.Drive;

/**
 * Add your docs here.
 */
public class DrivePathAction implements Action {

    private PathContainer mPathContainer;
    private Path mPath;
    private Drive mDrive = Drive.getInstance();
    private boolean mStopWhenDone;

    public DrivePathAction(PathContainer p, boolean stopWhenDone) {
        mPathContainer = p;
        mPath = mPathContainer.buildPath();
        mStopWhenDone = stopWhenDone;
    }

    public DrivePathAction(PathContainer p) {
        this(p, false);
    }
    
    @Override
    public void start() {
        mDrive.setDrivePath(mPath, mPathContainer.isReversed());
    }

    @Override
    public void update() {

    }

    @Override
    public boolean isFinished() {
        return mDrive.isDoneWithPath();
    }

    @Override
    public void done() {
        if(mStopWhenDone) {
            mDrive.setVelocity(new DriveSignal(0, 0), new DriveSignal(0, 0));
        }
    }
}
