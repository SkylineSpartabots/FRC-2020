/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.auto.actions;

import frc.robot.subsystems.Drive;

/**
 * Add your docs here.
 */
public class WaitForPathMarkerAction implements Action{

    private Drive mDrive = Drive.getInstance();
    private String mMarker;

    public WaitForPathMarkerAction(String marker) {
        mMarker = marker;
    }

    @Override
    public void start() {}

    @Override
    public void update() {}

    @Override
    public boolean isFinished() {
        return mDrive.hasPassedMarker(mMarker);
    }

    @Override
    public void done() {}
}
