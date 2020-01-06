/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;

import edu.wpi.first.wpilibj.Timer;
import frc.lib.control.LookAhead;
import frc.lib.control.Path;
import frc.lib.control.PathFollower;
import frc.lib.drivers.LazyTalonFX;
import frc.lib.drivers.PheonixUtil;
import frc.lib.drivers.TalonFXFactory;
import frc.lib.drivers.TalonFXUtil;
import frc.lib.geometry.Pose2d;
import frc.lib.geometry.Rotation2d;
import frc.lib.geometry.Twist2d;
import frc.lib.sensors.Navx;
import frc.lib.util.DriveSignal;
import frc.lib.util.TelemetryUtil;
import frc.lib.util.Util;
import frc.lib.util.TelemetryUtil.PrintStyle;
import frc.robot.Constants;
import frc.robot.Kinematics;
import frc.robot.Ports;
import frc.robot.RobotState;
import frc.robot.loops.ILooper;
import frc.robot.loops.Loop;


/**
 * Add your docs here.
 */
public class Drive extends Subsystem {

    private static Drive mInstance = null;

    public static Drive getInstance() {
        if(mInstance == null) {
            mInstance = new Drive();
        }
        return mInstance;
    }

    //hardware
    private final LazyTalonFX mLeftMaster, mLeftSlave, mRightMaster, mRightSlave;
    private final Navx mNavx;

    //hardware states
    private boolean mIsBrakeMode;
    private Rotation2d mGyroOffset = Rotation2d.identity();
    private PeriodicIO mPeriodicIO;

    //controllers
    private PathFollower mPathFollower;
    private Path mCurrentPath;

    //control states
    private DriveControlState mDriveControlState;

    private synchronized void configureMotorForDrive(LazyTalonFX falcon, InvertType inversion) {
        falcon.setInverted(inversion);
        PheonixUtil.checkError(falcon.configOpenloopRamp(0.3, Constants.kTimeOutMs), falcon.getName()
         + " failed to set open loop ramp rate", true);

        PheonixUtil.checkError(falcon.configClosedloopRamp(0.0, Constants.kTimeOutMs), falcon.getName()
         + " failed to set closed loop ramp rate", true);

        PheonixUtil.checkError(falcon.configVoltageCompSaturation(12.0, Constants.kTimeOutMs), falcon.getName() 
         + " failed to set voltage compensation", true);
  
        PheonixUtil.checkError(falcon.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 
            35, 0, 0)), falcon.getName() + " failed to set output current limit", true);

        PheonixUtil.checkError(falcon.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true,
             35, 0, 0)), falcon.getName() + " failed to set input current limit", true);
    }

    private synchronized void configureMasterForDrive(LazyTalonFX falcon, InvertType inversion, boolean sensorPhase) {
        configureMotorForDrive(falcon, inversion);
        PheonixUtil.checkError(falcon.configSelectedFeedbackCoefficient((2048/Constants.kWheelDiameter) / Math.PI, 0, Constants.kTimeOutMs),
            falcon.getName() + " failed to set sensor coeffecient", true);
        falcon.setSensorPhase(sensorPhase);
    }
    

    private Drive() {
        mPeriodicIO = new PeriodicIO();

        mLeftMaster = TalonFXFactory.createDefaultFalcon("Drive Left Master", Ports.DRIVE_LEFT_MASTER_ID);
        configureMasterForDrive(mLeftMaster, InvertType.None, false);

        mLeftSlave = TalonFXFactory.createSlaveFalcon("Drive Left Slave", Ports.DRIVE_LEFT_SLAVE_ID, Ports.DRIVE_LEFT_MASTER_ID);
        configureMotorForDrive(mLeftSlave, InvertType.FollowMaster);

        mRightMaster = TalonFXFactory.createDefaultFalcon("Drive Right Master", Ports.DRIVE_RIGHT_MASTER_ID);
        configureMasterForDrive(mRightMaster, InvertType.InvertMotorOutput, false);

        mRightSlave = TalonFXFactory.createSlaveFalcon("Drive Right Slave", Ports.DRIVE_RIGHT_SLAVE_ID, Ports.DRIVE_RIGHT_MASTER_ID);
        configureMotorForDrive(mRightSlave, InvertType.FollowMaster);

        mNavx = Navx.getInstance();

        mIsBrakeMode = false;
        setBrakeMode(true);

        setOpenLoop(DriveSignal.NEUTRAL);
    }


    private static class PeriodicIO {
        //inputs
        public double timestamp;
        public double left_position;
        public double right_position;
        public double left_distance;
        public double right_distance;
        public double left_velocity_per_100ms;
        public double right_velocity_per_100ms;
        public Rotation2d heading = Rotation2d.identity();
        public Pose2d error = Pose2d.identity();

        public double left_temperature;
        public double right_temperature;

        //outputs
        public double left_demand;
        public double right_demand;
        public double left_accel;
        public double right_accel;
        public double left_feedforward;
        public double right_feedforward;
    }

    @Override
    public synchronized void readPeriodicInputs() {
        mPeriodicIO.timestamp = Timer.getFPGATimestamp();

        double prevLeftPosition = mPeriodicIO.left_position;
        double prevRightPosition = mPeriodicIO.right_position;

        mPeriodicIO.left_position = mLeftMaster.getSelectedSensorPosition(0);
        mPeriodicIO.right_position = mRightMaster.getSelectedSensorPosition(0);

        double deltaLeftPosition = mPeriodicIO.left_position - prevLeftPosition;
        double deltaRightPosition = mPeriodicIO.right_position - prevRightPosition;

        mPeriodicIO.left_distance += deltaLeftPosition;
        mPeriodicIO.right_distance += deltaRightPosition;

        mPeriodicIO.left_velocity_per_100ms = mLeftMaster.getSelectedSensorVelocity(0);
        mPeriodicIO.right_velocity_per_100ms = mRightMaster.getSelectedSensorVelocity(0);

        mPeriodicIO.heading = Rotation2d.fromDegrees(mNavx.getHeading()).rotateBy(mGyroOffset);

        mPeriodicIO.left_temperature = mLeftMaster.getTemperature();
        mPeriodicIO.right_temperature = mRightMaster.getTemperature();
    }

    @Override
    public void writePeriodicOutputs() {
        if(mDriveControlState == DriveControlState.OPEN_LOOP) {
            mLeftMaster.set(ControlMode.PercentOutput, mPeriodicIO.left_demand,
                DemandType.ArbitraryFeedForward, 0.0);
            mRightMaster.set(ControlMode.PercentOutput, mPeriodicIO.right_demand,
                DemandType.ArbitraryFeedForward, 0.0);
        } else if (mDriveControlState == DriveControlState.PATH_FOLLOWING) {
            mLeftMaster.set(ControlMode.Velocity, mPeriodicIO.left_demand, DemandType.ArbitraryFeedForward,
                mPeriodicIO.left_feedforward + Constants.driveVelocityKd * mPeriodicIO.left_accel / 1023.0);
            
                mRightMaster.set(ControlMode.Velocity, mPeriodicIO.right_demand, DemandType.ArbitraryFeedForward,
                mPeriodicIO.right_feedforward + Constants.driveVelocityKd * mPeriodicIO.right_accel / 1023.0);
        }
    }

    @Override
    public void registerEnabledLoops(ILooper loop) {
        loop.register(new Loop() {

            @Override
            public void onStart(double timestamp) {
                synchronized(Drive.this) {
                    stop();
                    setBrakeMode(true);
                }                
            }

            @Override
            public void onLoop(double timestamp) {
                synchronized(Drive.this) {
                    handleFaults();
                    switch(mDriveControlState) {
                        case OPEN_LOOP:
                            break;
                        case PATH_FOLLOWING:
                            if(mPathFollower != null) {
                                updatePathFollower(timestamp);
                            }
                            break;
                        default:
                            TelemetryUtil.print("Drive in an unexpected control state", PrintStyle.ERROR, false);
                            break;
                    }
                }
            }

            @Override
            public void onStop(double timestamp) {
                stop();
            }
        });
    }

    public double getLeftEncoderPosition() {
        return mPeriodicIO.left_position;
    }

    public double getRightEncoderPosition() {
        return mPeriodicIO.right_position;
    }

    public double getLeftLinearVelocity() {
        return mPeriodicIO.left_velocity_per_100ms * 10.0;
    }

    public double getRightLinearVelocity() {
        return mPeriodicIO.right_velocity_per_100ms * 10.0;
    }

    public double getLinearVelocity() {
        return (getRightLinearVelocity() + getLeftLinearVelocity()) / 2.0;
    }

    public double getAverageDriveVelocityMagnitude() {
        return (Math.abs(getRightLinearVelocity()) + Math.abs(getLeftLinearVelocity())) / 2.0;
    }

    public double getAngularVelocity() {
        return (getRightLinearVelocity() - getLeftLinearVelocity()) / Constants.kDriveWheelTrackWidthInches;
    }

    public synchronized void setOpenLoop(DriveSignal signal) {
        if(mDriveControlState != DriveControlState.OPEN_LOOP) {
            setBrakeMode(true);
            setStatorCurrentLimit(0);
            mDriveControlState = DriveControlState.OPEN_LOOP;
        }

        mPeriodicIO.left_demand = signal.getLeft();
        mPeriodicIO.right_demand = signal.getRight();
        mPeriodicIO.left_feedforward = 0.0;
        mPeriodicIO.right_feedforward = 0.0;
    }

    public synchronized void setCurvatureDrive(double throttle, double curve, boolean quickTurn) {
        if(Util.epsilonEquals(throttle, 0.0, 0.04)) {
            throttle = 0.0;
        }

        if(Util.epsilonEquals(curve, 0.0, 0.04)) {
            curve = 0;
        }

        final double kCurveGain = 0.05;
        final double kCurveNonLinearity = 0.05;
        final double denominator = Math.sin(Math.PI / 2.0 * kCurveNonLinearity);

        if(!quickTurn) {
            curve = Math.sin(Math.PI / 2.0 * kCurveNonLinearity * curve);
            curve = Math.sin(Math.PI / 2.0 * kCurveNonLinearity * curve);
            curve = curve / (denominator * denominator) * Math.abs(throttle);
        }

        curve *= kCurveGain;
        DriveSignal signal = Kinematics.inverseKinematics(new Twist2d(throttle, 0.0, curve));
        double scaling_factor = Math.max(1.0, Math.max(Math.abs(signal.getLeft()), Math.abs(signal.getRight())));
        setOpenLoop(new DriveSignal(signal.getLeft() / scaling_factor, signal.getRight() / scaling_factor));
    }

    public synchronized void setVelocity(DriveSignal signal, DriveSignal feedforward) {
        if(mDriveControlState != DriveControlState.PATH_FOLLOWING) {
            setBrakeMode(true);
            setStatorCurrentLimit(35);
            mDriveControlState = DriveControlState.PATH_FOLLOWING;
        }

        mPeriodicIO.left_demand = signal.getLeft();
        mPeriodicIO.right_demand = signal.getRight();
        mPeriodicIO.left_feedforward = signal.getLeft();
        mPeriodicIO.right_feedforward = signal.getRight();
    }

    public synchronized void setDrivePath(Path path, boolean reversed) {
        if(mCurrentPath != path || mDriveControlState != DriveControlState.PATH_FOLLOWING) {
            RobotState.getInstance().resetDistanceDriven();
            mPathFollower = new PathFollower(path, reversed, new PathFollower.Parameters(
                new LookAhead(Constants.kMinLookAhead, Constants.kMaxLookAhead, Constants.kMinLookAheadSpeed, 
                    Constants.kMaxLookAheadSpeed), 
                Constants.kInertiaSteeringGain, Constants.kPathFollowingProfileKp,
                Constants.kPathFollowingProfileKi, Constants.kPathFollowingProfileKv,
                Constants.kPathFollowingProfileKffv, Constants.kPathFollowingProfileKffa,
                Constants.kPathFollowingProfileKs, Constants.kPathFollowingMaxVel,
                Constants.kPathFollowingMaxAccel, Constants.kPathFollowingGoalPosTolerance,
                Constants.kPathFollowingGoalVelTolerance, Constants.kPathStopSteeringDistance));
            mDriveControlState = DriveControlState.PATH_FOLLOWING;
            mCurrentPath = path;
        } else {
            setVelocity(DriveSignal.NEUTRAL, DriveSignal.NEUTRAL);
        }
    }

    public synchronized boolean isDoneWithPath() {
        if(mDriveControlState == DriveControlState.PATH_FOLLOWING && mPathFollower != null) {
            return mPathFollower.isFinished();
        }
        TelemetryUtil.print("Robot is not in a path following state", PrintStyle.NONE, false);
        return true;
    }

    public synchronized void forceDoneWithPath() {
        if(mDriveControlState == DriveControlState.PATH_FOLLOWING && mPathFollower != null) {
            mPathFollower.forceFinish();
        }
        TelemetryUtil.print("Robot is not in a path following state", PrintStyle.NONE, false); 
    }

    private void updatePathFollower(double timestamp) {
        if(mDriveControlState == DriveControlState.PATH_FOLLOWING) {
            RobotState robot_state = RobotState.getInstance();
            Pose2d field_to_vehicle = robot_state.getLatestFieldToVehicle().getValue();
            Twist2d command = mPathFollower.update(timestamp, field_to_vehicle, robot_state.getDistanceDriven(),
                 robot_state.getPredictedVelocity().dx);
            if(!mPathFollower.isFinished()) {
                DriveSignal setpoint = Kinematics.inverseKinematics(command);
                setVelocity(setpoint, new DriveSignal(0, 0));
            } else {
                if(!mPathFollower.isForceFinished()) {
                    setVelocity(new DriveSignal(0, 0), new DriveSignal(0, 0));
                }
            }
        } else {
            TelemetryUtil.print("Drive is not in a path following state", PrintStyle.WARNING, false);
        }
    }

    public synchronized boolean hasPassedMarker(String marker) {
        if(mDriveControlState == DriveControlState.PATH_FOLLOWING && mPathFollower != null) {
            return mPathFollower.hasPassedMarker(marker);
        } else {
            TelemetryUtil.print("Drive is not in a path following state", PrintStyle.NONE, false);
            return false;
        }
    }

    public synchronized Rotation2d getHeading() {
        return mPeriodicIO.heading;
    }

    public synchronized void setHeading(Rotation2d heading) {
        mGyroOffset = heading.rotateBy(Rotation2d.fromDegrees(mNavx.getHeading()).inverse());
        mPeriodicIO.heading = heading;
    } 

    public synchronized void resetEncoders() {
        PheonixUtil.checkError(mLeftMaster.setSelectedSensorPosition(0, 0, Constants.kTimeOutMs),
            mLeftMaster.getName() + " failed to reset encoder", true);
        PheonixUtil.checkError(mRightMaster.setSelectedSensorPosition(0, 0, Constants.kTimeOutMs),
            mRightMaster.getName() + " failed to reset encoder", true);
    }


    public synchronized void setBrakeMode(boolean enableBrake) {
        if(enableBrake != mIsBrakeMode) {
            NeutralMode mode = enableBrake ? NeutralMode.Brake : NeutralMode.Brake;
            mLeftMaster.setNeutralMode(mode);
            mLeftSlave.setNeutralMode(mode);
            mRightMaster.setNeutralMode(mode);
            mRightSlave.setNeutralMode(mode);
            mIsBrakeMode = enableBrake;
        }
    }

    public synchronized void setStatorCurrentLimit(int amps) {
        TalonFXUtil.setStatorCurrentLimit(mLeftMaster, amps);
        TalonFXUtil.setStatorCurrentLimit(mLeftSlave, amps);
        TalonFXUtil.setStatorCurrentLimit(mRightMaster, amps);
        TalonFXUtil.setStatorCurrentLimit(mRightSlave, amps);
    }

    public synchronized void handleFaults() {
        TalonFXUtil.checkSensorFaults(mLeftMaster);
        TalonFXUtil.checkSlaveFaults(mLeftSlave, Ports.DRIVE_LEFT_MASTER_ID);
        TalonFXUtil.checkSensorFaults(mRightMaster);
        TalonFXUtil.checkSlaveFaults(mRightSlave, Ports.DRIVE_RIGHT_MASTER_ID);
    }

    private enum DriveControlState {
        OPEN_LOOP,
        PATH_FOLLOWING;
    }

    @Override
    public void zeroSensors() {
        setHeading(Rotation2d.identity());
        resetEncoders();
    }

    @Override
    public void stop() {
        setOpenLoop(DriveSignal.NEUTRAL);
    }

    @Override
    public boolean checkSystem() {
        // TODO Auto-generated method stub
        return false;
    }

    @Override
    public void outputTelemetry() {
        // TODO Auto-generated method stub

    }


    
}