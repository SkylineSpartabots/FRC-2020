package frc.robot.auto.modes;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import frc.lib.util.DriveSignal;
import frc.robot.Constants;
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

public class PowerPort extends AutoModeBase {

    /**
     * Set driverstation to disable the robot after I think 1 minute of operation
     * (check rules for allowed operation time)
     */

    private Shooter mShooter = Shooter.getInstance();
    private Intake mIntake = Intake.getInstance();
    private Hopper mHopper = Hopper.getInstance();
    private Drive mDrive = Drive.getInstance();
    private Superstructure mSuperstructure = Superstructure.getInstance();
    private double startTime;

    @Override
    protected void routine() throws AutoModeEndedException {
        mShooter.setOpenLoop(0.0);
        mDrive.setOpenLoop(new DriveSignal(0, 0));
        mIntake.conformToState(IntakeControlState.DOWN);
        startTime = Timer.getFPGATimestamp();

        while (Timer.getFPGATimestamp() - startTime <= 60) {
            shoot(Constants.kBlueZonePowerPortRPM);
            //blule to reintroduction
            runAction(new PerfectlyStraightDriveAction(Rotation2d.fromDegrees(0), 2.75, 0.5));
            mDrive.setOpenLoop(new DriveSignal(0, 0));
    
            reintroduction();

            //reintroduction to blues
            runAction(new PerfectlyStraightDriveAction(Rotation2d.fromDegrees(0), 2.79, -0.5));
            mDrive.setOpenLoop(new DriveSignal(0, 0));
        }

        mIntake.conformToState(IntakeControlState.OFF);
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
    private void shoot(int currentRPM) throws AutoModeEndedException {
        runAction(new WaitAction(1));
        mSuperstructure.autoShootBalls(3, currentRPM, 0.5);
        //runAction(new WaitAction(5));
        runAction(new WaitForRequestsAction());
       /* mSuperstructure.autoShootBalls(1, currentRPM, 1);
        //runAction(new WaitAction(5));
        runAction(new WaitForRequestsAction());
        mSuperstructure.autoShootBalls(1, currentRPM, 1);
        runAction(new WaitForRequestsAction());*/

        mShooter.shootAtSetRpm(Constants.kBlueZonePowerPortRPM);
        mHopper.conformToState(HopperControlState.OFF);
    }
    
}
