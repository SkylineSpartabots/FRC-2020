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
        
        //vishal, 5, 6
        /**
         * Limelight exposure level
         * Green: 2
         * Yellow: 2
         * Blue: 7
         * Red: 19
         */

        mShooter.setOpenLoop(0.0);
        mDrive.setOpenLoop(new DriveSignal(0, 0));
        mIntake.conformToState(IntakeControlState.DOWN);


        shoot(Constants.kGreenZoneRPM, Constants.kYellowZoneRPM);
        
        
        //Green zone to reintroduction
        runAction(new PerfectlyStraightDriveAction(Rotation2d.fromDegrees(0.0), 6, 0.15));
        mDrive.setOpenLoop(new DriveSignal(0, 0));
        
        //runAction(new WaitAction(1));
        reintroduction();


        //Reintroduction to yellow zone
        runAction(new PerfectlyStraightDriveAction(Rotation2d.fromDegrees(0), 4.35, -0.15));
        mDrive.setOpenLoop(new DriveSignal(0, 0));

       shoot(Constants.kYellowZoneRPM, Constants.kBlueZoneRPM);
       //runAction(new WaitAction(5));


        //yellow zone to reintroduction
        runAction(new PerfectlyStraightDriveAction(Rotation2d.fromDegrees(0.0), 4.45, 0.15));
        mDrive.setOpenLoop(new DriveSignal(0, 0));
        
        reintroduction();
     //runAction(new WaitAction(1));

        //reintroduction to blue zone
        runAction(new PerfectlyStraightDriveAction(Rotation2d.fromDegrees(0), 2.95, -0.15));
        mDrive.setOpenLoop(new DriveSignal(0, 0));
        
        shoot(Constants.kBlueZoneRPM, Constants.kRedZoneRPM);
       
        //blue zone to reintroduction
        runAction(new PerfectlyStraightDriveAction(Rotation2d.fromDegrees(0), 3.05, 0.15));
        mDrive.setOpenLoop(new DriveSignal(0, 0));

        reintroduction();

        //reintroduction to blue zone
        runAction(new PerfectlyStraightDriveAction(Rotation2d.fromDegrees(0), 2.95, -0.15));
        mDrive.setOpenLoop(new DriveSignal(0, 0));
        
        shoot(Constants.kBlueZoneRPM, Constants.kRedZoneRPM);
       
        //blue zone to reintroduction
        runAction(new PerfectlyStraightDriveAction(Rotation2d.fromDegrees(0), 3.05, 0.15));
        mDrive.setOpenLoop(new DriveSignal(0, 0));

        reintroduction();


        //reintroduction to red zone
        runAction(new PerfectlyStraightDriveAction(Rotation2d.fromDegrees(0), 1.6, -0.15));
        mDrive.setOpenLoop(new DriveSignal(0, 0));
        
        shoot(Constants.kRedZoneRPM, 0);
        
    /*    //red to reintroduction
        runAction(new PerfectlyStraightDriveAction(Rotation2d.fromDegrees(0), 1.6, 0.15));
        mDrive.setOpenLoop(new DriveSignal(0, 0));

        reintroduction();*/


        mIntake.conformToState(IntakeControlState.DOWN);
        mHopper.conformToState(HopperControlState.OFF);
        mShooter.setOpenLoop(0);
        mDrive.setOpenLoop(new DriveSignal(0, 0));

    }


    /**
     * Intakes until a ball is recognized by the beambreak
     * @throws AutoModeEndedException
     */
    private void reintroduction() throws AutoModeEndedException {
        mSuperstructure.intakeUntilBallDetected();
        runAction(new WaitForRequestsAction());
        mHopper.conformToState(HopperControlState.OFF);
    }

    /**
     * shoots 3 balls at current rpm
     * then ramps shooter for nextrpm
     */
    private void shoot(int currentRPM, int nextRPM) throws AutoModeEndedException {
        runAction(new WaitAction(1));
        mSuperstructure.autoShootBalls(1, currentRPM, 2);
        runAction(new WaitAction(3));
        runAction(new WaitForRequestsAction());
        mSuperstructure.autoShootBalls(1, currentRPM, 3);
        runAction(new WaitAction(3));
        runAction(new WaitForRequestsAction());
        mSuperstructure.autoShootBalls(1, currentRPM, 2);
        runAction(new WaitForRequestsAction());

        mShooter.setOpenLoop(0.0);
        mHopper.conformToState(HopperControlState.OFF);
    }
    
}
