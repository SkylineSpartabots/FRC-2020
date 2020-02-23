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
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Superstructure;

/**
 * Add your docs here.
 */
public class PositionThreeThiefAuto extends AutoModeBase {
    DrivePathAction pathToTrench;
    DrivePathAction pathToMid;
    RunOnceAction intakeBalls;
    RunOnceAction shootBalls;
    Intake mIntake;
    Superstructure s;
    public PositionThreeThiefAuto() {
        mIntake = Intake.getInstance();
        s = Superstructure.getInstance();
        pathToTrench = new DrivePathAction(paths.startPositionThreeToOpponentTrench, false);
        pathToMid = new DrivePathAction(paths.opponentTrenchToMidField, true);
        intakeBalls = new RunOnceAction() {

			@Override
			public void runOnce() {
				s.intakeForSecondsThenStop(4);
			}
            
        };
        shootBalls = new RunOnceAction() {

			@Override
			public void runOnce() {
                s.autoShootSequence();
			}

        };
    }
    @Override
    protected void routine() throws ModeEndedException {
        runAction(pathToTrench);
        runAction(intakeBalls); //Theoretically, this should be run with the first path
        runAction(pathToMid);
        runAction(shootBalls);
    }
}
