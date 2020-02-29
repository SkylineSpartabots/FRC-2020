/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.auto.modes;

import frc.lib.util.DriveSignal;
import frc.robot.auto.AutoModeEndedException;
import frc.robot.auto.actions.DrivePathAction;
import frc.robot.auto.actions.WaitAction;
import frc.robot.subsystems.Drive;

/**
 * Add your docs here.
 */
public class TenBallDepotSide extends AutoModeBase {

    @Override
    protected void routine() throws AutoModeEndedException {
        Drive.getInstance().setOpenLoop(new DriveSignal(0, 0));
        runAction(new DrivePathAction(paths.startPositionOneToTrenchEnd, true));
        runAction(new WaitAction(1));
        runAction(new DrivePathAction(paths.trenchEndToShootLocation, true));
    }
}
