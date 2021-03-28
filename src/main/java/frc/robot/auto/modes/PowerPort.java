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

    @Override
    protected void routine() throws AutoModeEndedException {
        
        for(int i = 0; i < 20; i++) {
            shoot(1000);
            runAction(new PerfectlyStraightDriveAction(Rotation2d.fromDegrees(0.0), 1, 0.1));
            mDrive.setOpenLoop(new DriveSignal(0, 0));
            reintroduction();
            runAction(new PerfectlyStraightDriveAction(Rotation2d.fromDegrees(0.0), 1, -0.1));
            mDrive.setOpenLoop(new DriveSignal(0, 0));
        }

        mIntake.conformToState(IntakeControlState.OFF);
        mHopper.conformToState(HopperControlState.OFF);
        mShooter.setOpenLoop(0);
        mDrive.setOpenLoop(new DriveSignal(0, 0));
        
    }

    private void reintroduction() throws AutoModeEndedException {
        mSuperstructure.intakeUntilBallDetected();
        runAction(new WaitForRequestsAction());
    }

    private void shoot(int currentRPM) throws AutoModeEndedException {
        mSuperstructure.autoShootBalls(3, currentRPM, 0);
        runAction(new WaitForRequestsAction());
    }
    
}
