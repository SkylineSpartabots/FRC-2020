package frc.robot.auto.modes;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import frc.lib.util.DriveSignal;
import frc.robot.Constants;
import frc.robot.Robot;
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

public class InterstellarAccuracy extends AutoModeBase{

    /**
     * TODO: add another cycle at the "sweet spot" position
     */
    
    private Shooter mShooter = Shooter.getInstance();
    private Intake mIntake = Intake.getInstance();
    private Hopper mHopper = Hopper.getInstance();
    private Drive mDrive = Drive.getInstance();
    private Superstructure mSuperstructure = Superstructure.getInstance();

    @Override
    protected void routine() throws AutoModeEndedException {
        mShooter.setOpenLoop(0.0);
        mDrive.setOpenLoop(new DriveSignal(0, 0));

        
        shoot(Constants.kGreenZoneRPM, Constants.kYellowZoneRPM);
        
        
        //Green zone to reintroduction
        runAction(new PerfectlyStraightDriveAction(Rotation2d.fromDegrees(0.0), 5.2, 0.15));
        mDrive.setOpenLoop(new DriveSignal(0, 0));
        
        
        reintroduction();


        //Reintroduction to yellow zone
        runAction(new PerfectlyStraightDriveAction(Rotation2d.fromDegrees(0), 4.5, -0.15));
        mDrive.setOpenLoop(new DriveSignal(0, 0));

        shoot(Constants.kYellowZoneRPM, Constants.kBlueZoneRPM);



        //yellow zone to reintroduction
        runAction(new PerfectlyStraightDriveAction(Rotation2d.fromDegrees(0.0), 4.5, 0.15));
        mDrive.setOpenLoop(new DriveSignal(0, 0));
        
        reintroduction();


        //reintroduction to blue zone
        runAction(new PerfectlyStraightDriveAction(Rotation2d.fromDegrees(0), 3.0, -0.15));
        mDrive.setOpenLoop(new DriveSignal(0, 0));
        
        shoot(Constants.kBlueZoneRPM, Constants.kRedZoneRPM);
       
        //blue zone to reintroduction
        runAction(new PerfectlyStraightDriveAction(Rotation2d.fromDegrees(0), 3.0, 0.15));
        mDrive.setOpenLoop(new DriveSignal(0, 0));

        reintroduction();


        //reintroduction to red zone
        runAction(new PerfectlyStraightDriveAction(Rotation2d.fromDegrees(0), -1.5, -0.15));
        mDrive.setOpenLoop(new DriveSignal(0, 0));
        
        shoot(Constants.kRedZoneRPM, 0);
        
        mIntake.conformToState(IntakeControlState.OFF);
        mHopper.conformToState(HopperControlState.OFF);
        mShooter.setOpenLoop(0);
        mDrive.setOpenLoop(new DriveSignal(0, 0));

    }



    private void reintroduction() throws AutoModeEndedException {
        mSuperstructure.intakeUntilBallDetected();
        runAction(new WaitForRequestsAction());
    }

    private void shoot(int currentRPM, int nextRPM) throws AutoModeEndedException {
        runAction(new WaitAction(0.25));
        mSuperstructure.autoShootBalls(3, currentRPM, 0);
        runAction(new WaitForRequestsAction());

        mShooter.shootAtSetRpm(nextRPM);
    }
    
}
