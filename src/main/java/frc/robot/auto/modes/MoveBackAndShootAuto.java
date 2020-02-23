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
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Superstructure;

/**
 * Add your docs here.
 */
public class MoveBackAndShootAuto extends AutoModeBase {
    RunOnceAction moveBackAndShoot;
    Drive mDrive;
    Superstructure s;
    public MoveBackAndShootAuto() {
        mDrive = Drive.getInstance();
        s = Superstructure.getInstance();
        moveBackAndShoot = new RunOnceAction() {

			@Override
			public void runOnce() {
                s.autoShootSequence();
                mDrive.timeDriveRequest(new DriveSignal(-0.25,-0.25), 2);
			}
            
        };
    }
    @Override
    protected void routine() throws ModeEndedException {
        runAction(moveBackAndShoot);
    }
}
