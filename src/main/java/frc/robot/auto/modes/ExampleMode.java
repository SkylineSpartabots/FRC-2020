/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.auto.modes;

import frc.robot.auto.AutoModeEndedException;
import frc.robot.auto.actions.DrivePathAction;
import frc.robot.paths.MoveAndTurnPath;
import frc.robot.paths.MoveTestPath;
import frc.robot.paths.TurnTestPath;

/**
 * Testing pathing
 */
public class ExampleMode extends AutoModeBase {

    @Override
    protected void routine() throws AutoModeEndedException {
        runAction(new DrivePathAction(new MoveTestPath()));
        runAction(new DrivePathAction(new TurnTestPath()));
        runAction(new DrivePathAction(new MoveAndTurnPath()));
    }
}
