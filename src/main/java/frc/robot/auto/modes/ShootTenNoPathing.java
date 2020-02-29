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
public class ShootTenNoPathing extends AutoModeBase {


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
        mIntake.conformToState(IntakeControlState.INTAKE);
        runAction(new PerfectlyStraightDriveAction(Rotation2d.fromDegrees(0.0), 3.05, 0.75));
        runAction(new PerfectlyStraightDriveAction(Rotation2d.fromDegrees(60.0), 0.4, 0.5));

        runAction(new PerfectlyStraightDriveAction(Rotation2d.fromDegrees(60.0), 0.4, -0.5));
        runAction(new PerfectlyStraightDriveAction(Rotation2d.fromDegrees(0.0), 3.00, -0.6));
        
        mDrive.setOpenLoop(new DriveSignal(0, 0));

        mSuperstructure.autoShootBalls(5, 4700, 0);
        runAction(new WaitForRequestsAction());
        runAction(new WaitAction(0.15));

        mHopper.conformToState(HopperControlState.OFF);
        mShooter.setOpenLoop(0.1);
        mIntake.conformToState(IntakeControlState.INTAKE);

        runAction(new PerfectlyStraightDriveAction(Rotation2d.fromDegrees(-64.0), 1.28, 0.50));
        runAction(new PerfectlyStraightDriveAction(Rotation2d.fromDegrees(0.0), 1.0, 0.50));
        runAction(new PerfectlyStraightDriveAction(Rotation2d.fromDegrees(0.0), 3.2, 0.8));
        mShooter.shootAtSetRpm(4600);
        runAction(new PerfectlyStraightDriveAction(Rotation2d.fromDegrees(0.0), 2.5, -0.8));
        runAction(new PerfectlyStraightDriveAction(Rotation2d.fromDegrees(-15), 0.3, -0.65));
        mDrive.setOpenLoop(new DriveSignal(0.0, 0.0));
        
        

        mSuperstructure.autoShootBalls(5, 4600, 0.4);
        runAction(new WaitForRequestsAction());

        mHopper.conformToState(HopperControlState.OFF);
        mIntake.conformToState(IntakeControlState.IDLE_WHILE_DEPLOYED);
        mShooter.setOpenLoop(0.0);

    }
}
