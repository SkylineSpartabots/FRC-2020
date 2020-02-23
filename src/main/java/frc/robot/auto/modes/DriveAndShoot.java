/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.auto.modes;

import frc.robot.auto.ModeEndedException;
import frc.robot.auto.actions.DrivePathAction;
import frc.robot.auto.actions.RunOnceAction;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Superstructure;

/**
 * Add your docs here.
 */
public class DriveAndShoot extends AutoModeBase {
    DrivePathAction driveToLocation;
    RunOnceAction shootBalls;
    Drive mDrive;
    Superstructure s;
    public DriveAndShoot() {
        driveToLocation = new DrivePathAction(paths.startPositionThreeToMidField, true);
        shootBalls = new RunOnceAction() {

			@Override
			public void runOnce() {
                s.autoShootSequence();
			}
            
        };
    }
    @Override
    protected void routine() throws ModeEndedException {
        runAction(driveToLocation);
        runAction(shootBalls);
    }
}
