/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.auto.modes.tests;

import frc.robot.auto.ModeEndedException;
import frc.robot.auto.actions.LambdaAction;
import frc.robot.auto.modes.AutoModeBase;
import frc.robot.subsystems.Drive;

/**
 * Add your docs here.
 */
public class DriveConfiguration extends AutoModeBase {

    @Override
    protected void routine() throws ModeEndedException {
        runAction(new LambdaAction(() -> Drive.getInstance().testDriveConfiguration()));

    }


   
}
