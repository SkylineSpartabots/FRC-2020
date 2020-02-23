/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.auto.modes;

import frc.lib.util.DriveSignal;
import frc.robot.auto.ModeEndedException;
import frc.robot.auto.actions.RunOnceAction;
import frc.robot.subsystems.Drive;;

/**
 * Add your docs here.
 */
public class MoveBackAuto extends AutoModeBase {
    RunOnceAction moveBack;
    Drive mDrive;
    public MoveBackAuto() {
        mDrive = Drive.getInstance();
        moveBack = new RunOnceAction() {

			@Override
			public void runOnce() {
                mDrive.timeDriveRequest(new DriveSignal(-0.25,-0.25), 2);
			}
            
        };
    }
    @Override
    protected void routine() throws ModeEndedException {
        runAction(moveBack);
    }
}
