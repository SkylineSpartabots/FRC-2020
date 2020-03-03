/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.auto.modes;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import frc.lib.util.DriveSignal;
import frc.robot.auto.AutoModeEndedException;
import frc.robot.auto.actions.PerfectlyStraightDriveAction;
import frc.robot.auto.actions.WaitAction;
import frc.robot.auto.actions.WaitForRequestsAction;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Hopper.HopperControlState;
import frc.robot.subsystems.Intake.IntakeControlState;

/**
 * Add your docs here.
 */
public class TenBallDepotSide extends AutoModeBase {

    private Shooter mShooter = Shooter.getInstance();
    private Intake mIntake = Intake.getInstance();
    private Hopper mHopper = Hopper.getInstance();
    private Drive mDrive = Drive.getInstance();
    private Superstructure mSuperstructure = Superstructure.getInstance();

    
    @Override
    protected void routine() throws AutoModeEndedException {
        mShooter.setOpenLoop(0.0);
        mDrive.setOpenLoop(new DriveSignal(0, 0));

        mShooter.shootAtSetRpm(4700);
        mIntake.conformToState(IntakeControlState.STORE);
        mHopper.conformToState(HopperControlState.OFF);

        runAction(new PerfectlyStraightDriveAction(Rotation2d.fromDegrees(0.0), 3.05, 0.85));

        mIntake.conformToState(IntakeControlState.INTAKE);
        mHopper.conformToState(HopperControlState.SENSORED_INTAKE);

        runAction(new PerfectlyStraightDriveAction(Rotation2d.fromDegrees(60.0), 0.4, 0.6));

        runAction(new PerfectlyStraightDriveAction(Rotation2d.fromDegrees(60.0), 0.4, -0.6));
        runAction(new PerfectlyStraightDriveAction(Rotation2d.fromDegrees(0.0), 2.50, -0.9));
        mSuperstructure.autoShootNoAlign(5, 4700, 0.0);
        runAction(new PerfectlyStraightDriveAction(Rotation2d.fromDegrees(0.0), 0.50, -0.75));
        
        mDrive.setOpenLoop(new DriveSignal(0, 0));

        runAction(new WaitForRequestsAction());
        runAction(new WaitAction(0.15));

        mShooter.setOpenLoop(0.2);
        mIntake.conformToState(IntakeControlState.INTAKE);
        mHopper.conformToState(HopperControlState.SENSORED_INTAKE);

        runAction(new PerfectlyStraightDriveAction(Rotation2d.fromDegrees(-64.0), 1.08, 0.8));
        runAction(new PerfectlyStraightDriveAction(Rotation2d.fromDegrees(0.0), 4.05, 0.8));
        runAction(new PerfectlyStraightDriveAction(Rotation2d.fromDegrees(0.0), 0.82, 0.7));


        runAction(new PerfectlyStraightDriveAction(Rotation2d.fromDegrees(0.0), 0.82, -0.7));
        mShooter.shootAtSetRpm(4750);
        runAction(new PerfectlyStraightDriveAction(Rotation2d.fromDegrees(0.0), 2.35, -0.8));
        runAction(new PerfectlyStraightDriveAction(Rotation2d.fromDegrees(-14.0), 0.3, -0.75));
        mDrive.setOpenLoop(new DriveSignal(0.0, 0.0));
        
    
        mSuperstructure.autoShootNoAlign(5, 4750, 0.0);
        runAction(new WaitForRequestsAction());

        mHopper.conformToState(HopperControlState.OFF);
        mIntake.conformToState(IntakeControlState.IDLE_WHILE_DEPLOYED);
        mShooter.setOpenLoop(0.0);

    }
}
