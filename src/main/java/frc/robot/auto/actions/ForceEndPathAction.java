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
public class ForceEndPathAction extends RunOnceAction {

    @Override
    public void runOnce() {
        Drive.getInstance().forceDoneWithPath();

    }

}
