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
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
        if (mInstance == null) {
            mInstance = new Drive();
        }
        return mInstance;
    }

    // hardware
    private final LazyTalonFX mLeftMaster, mLeftSlave, mRightMaster, mRightSlave;
    private final Navx mNavx;

    // hardware states
    private boolean mIsBrakeMode;
    private Rotation2d mGyroOffset = Rotation2d.identity();
    private PeriodicIO mPeriodicIO;

    // controllers
    private PathFollower mPathFollower;
    private Path mCurrentPath;
    private DifferentialDrive mOpenLoopController;

    // control states
    private DriveControlState mDriveControlState;

    /**
     * sets the motor's inversion, open loop ramp, closed loop ramp, voltage comp sat, and current limits
     * @param falcon the falcon being set up
     * @param inversion inverted state
     */
    private synchronized void configureMotorForDrive(LazyTalonFX falcon, InvertType inversion) {
        falcon.setInverted(inversion);
        PheonixUtil.checkError(falcon.configOpenloopRamp(0.3, Constants.kTimeOutMs),
                falcon.getName() + " failed to set open loop ramp rate", true);

        PheonixUtil.checkError(falcon.configClosedloopRamp(0.0, Constants.kTimeOutMs),
                falcon.getName() + " failed to set closed loop ramp rate", true);

        PheonixUtil.checkError(falcon.configVoltageCompSaturation(12.0, Constants.kTimeOutMs),
                falcon.getName() + " failed to set voltage compensation", true);

        PheonixUtil.checkError(falcon.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 35, 0, 0)),
                falcon.getName() + " failed to set output current limit", true);

        PheonixUtil.checkError(falcon.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 35, 0, 0)),
                falcon.getName() + " failed to set input current limit", true);
    }

    /**
     * configures the sensor phase and feedback coefficient for a master motor
     * @param falcon the falcon being set up
     * @param inversion inverted state
     * @param sensorPhase is sensor in same direction as motor
     */
    private synchronized void configureMasterForDrive(LazyTalonFX falcon, InvertType inversion, boolean sensorPhase) {
        configureMotorForDrive(falcon, inversion);
        PheonixUtil.checkError(falcon.configSelectedFeedbackCoefficient((2048 / Constants.kWheelDiameter) / Math.PI, 0,
                Constants.kTimeOutMs), falcon.getName() + " failed to set sensor coeffecient", true);
        falcon.setSensorPhase(sensorPhase);
    }

    private Drive() {
        mPeriodicIO = new PeriodicIO();

        mLeftMaster = TalonFXFactory.createDefaultFalcon("Drive Left Master", Ports.DRIVE_LEFT_MASTER_ID);
        configureMasterForDrive(mLeftMaster, InvertType.None, false);

        mLeftSlave = TalonFXFactory.createSlaveFalcon("Drive Left Slave", Ports.DRIVE_LEFT_SLAVE_ID,
                Ports.DRIVE_LEFT_MASTER_ID);
        configureMotorForDrive(mLeftSlave, InvertType.FollowMaster);

        mRightMaster = TalonFXFactory.createDefaultFalcon("Drive Right Master", Ports.DRIVE_RIGHT_MASTER_ID);
        configureMasterForDrive(mRightMaster, InvertType.InvertMotorOutput, false);

        mRightSlave = TalonFXFactory.createSlaveFalcon("Drive Right Slave", Ports.DRIVE_RIGHT_SLAVE_ID,
                Ports.DRIVE_RIGHT_MASTER_ID);
        configureMotorForDrive(mRightSlave, InvertType.FollowMaster);

        mNavx = Navx.getInstance();

        mIsBrakeMode = false;
        setBrakeMode(true);

        setOpenLoop(DriveSignal.NEUTRAL);
    }

    /**
     * tracks the inputs and outputs of the drive train
     */
    private static class PeriodicIO {
        // inputs
        public double timestamp;
        /**position of left encoder (raw) */
        public double left_position;
        /**position of right encoder (raw) */
        public double right_position;
        /**absolute change in left encoder distance (raw) */
        public double left_distance;
        /**absolute change in right encoder distance (raw) */
        public double right_distance;
        /**left encoder distance (raw) per 100ms */
        public double left_velocity_per_100ms;
        /**right encoder distance (raw) per 100ms */
        public double right_velocity_per_100ms;
        /**current direction of robot using navx */
        public Rotation2d heading = Rotation2d.identity();
        public Pose2d error = Pose2d.identity();

        public double left_master_temperature;
        public double left_slave_temperature;
        public double right_master_temperature;
        public double right_slave_temperature;

        // outputs
        public double left_demand;
        public double right_demand;
        public double left_accel;
        public double right_accel;
        public double left_feedforward;
        public double right_feedforward;
    }

    /**
     * reads encoder position, change in position, velocity, heading, and temperature
     */
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

        mPeriodicIO.left_master_temperature = mLeftMaster.getTemperature();
        mPeriodicIO.left_slave_temperature = mLeftSlave.getTemperature();
        mPeriodicIO.right_master_temperature = mRightMaster.getTemperature();
        mPeriodicIO.right_slave_temperature = mRightSlave.getTemperature();
    }

    /**
     * if open loop: sets output from periodic io
     * if path following: sets desired velocity to motors from periodic io
     * arbitrary feed forward adds the fourth parameter to the output
     */
    @Override
    public void writePeriodicOutputs() {
        if (mDriveControlState == DriveControlState.OPEN_LOOP) {
            mLeftMaster.set(ControlMode.PercentOutput, mPeriodicIO.left_demand, DemandType.ArbitraryFeedForward, 0.0);
            mRightMaster.set(ControlMode.PercentOutput, mPeriodicIO.right_demand, DemandType.ArbitraryFeedForward, 0.0);
        } else if (mDriveControlState == DriveControlState.PATH_FOLLOWING) {
            mLeftMaster.set(ControlMode.Velocity, mPeriodicIO.left_demand, DemandType.ArbitraryFeedForward,
                    mPeriodicIO.left_feedforward + Constants.driveVelocityKd * mPeriodicIO.left_accel / 1023.0);

            mRightMaster.set(ControlMode.Velocity, mPeriodicIO.right_demand, DemandType.ArbitraryFeedForward,
                    mPeriodicIO.right_feedforward + Constants.driveVelocityKd * mPeriodicIO.right_accel / 1023.0);
        }
    }

    /**
     * registers drive train's loop to subsystem manager
     */
    @Override
    public void registerEnabledLoops(ILooper loop) {
        loop.register(new Loop() {

            @Override
            public void onStart(double timestamp) {
                synchronized (Drive.this) {
                    stop();
                    setBrakeMode(true);
                }
            }

            @Override
            public void onLoop(double timestamp) {
                synchronized (Drive.this) {
                    handleFaults();
                    switch (mDriveControlState) {
                    case OPEN_LOOP:
                        break;
                    case PATH_FOLLOWING:
                        if (mPathFollower != null) {
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

    /**
     * @return raw position of left encoder
     */
    public double getLeftEncoderPosition() {
        return mPeriodicIO.left_position;
    }

    /**
     * 
     * @return raw position of right encoder
     */
    public double getRightEncoderPosition() {
        return mPeriodicIO.right_position;
    }

    /**
     * @return left encoder distance (raw) per second
     */
    public double getLeftLinearVelocity() {
        return mPeriodicIO.left_velocity_per_100ms * 10.0;
    }

    /**
     * 
     * @return right encoder distance (raw) per second
     */
    public double getRightLinearVelocity() {
        return mPeriodicIO.right_velocity_per_100ms * 10.0;
    }

    /**
     * @return average linear distance (raw) per second with both sides
     */
    public double getLinearVelocity() {
        return (getRightLinearVelocity() + getLeftLinearVelocity()) / 2.0;
    }

    /**
     * 
     * @return absolute value of linear average drive velocity
     */
    public double getAverageDriveVelocityMagnitude() {
        return (Math.abs(getRightLinearVelocity()) + Math.abs(getLeftLinearVelocity())) / 2.0;
    }

    /**
     * 
     * @return angular velocity of bot in degrees per second
     */
    public double getAngularVelocity() {
        return (getRightLinearVelocity() - getLeftLinearVelocity()) / Constants.kDriveWheelTrackWidthInches;
    }


    /**
     * 
     * Sets a drive signal to the motors. If the robot is not in a driving state, it will turn break mode on.
     * @param signal drive signal to set percent power to motors
     */
    public synchronized void setOpenLoop(DriveSignal signal) {
        if (mDriveControlState != DriveControlState.OPEN_LOOP) {
            setBrakeMode(true);
            // setStatorCurrentLimit(0);
            mDriveControlState = DriveControlState.OPEN_LOOP;
        }

        mPeriodicIO.left_demand = signal.getLeft();
        mPeriodicIO.right_demand = signal.getRight();
        mPeriodicIO.left_feedforward = 0.0;
        mPeriodicIO.right_feedforward = 0.0;
    }

    private double mQuickStopAccumulator;
    /**
     * sets power to the robot in such a way to create a curved drive
     * @param throttle forward amount
     * @param curve turning amount
     * @param quickTurn should the robot turn quickly
     */
    public synchronized void setCurvatureDrive(double throttle, double curve, boolean quickTurn) {
        throttle = Util.limit(throttle, -1.0, 1.0);
        throttle = Util.deadBand(throttle, 0.04);

        curve = Util.limit(curve, -1.0, 1.0);
        curve = Util.deadBand(curve, 0.04);

        double angularPower;
        boolean overPower;
    
        if (quickTurn) {
            if (Math.abs(throttle) < Constants.kQuickStopThreshold) {
                mQuickStopAccumulator = (1 - Constants.kQuickStopAlpha) * mQuickStopAccumulator
                        + Constants.kQuickStopAlpha * Util.limit(curve, -1.0, 1.0) * 2;
            }
            overPower = true;
            angularPower = curve;
        } else {
            overPower = false;
            angularPower = Math.abs(throttle) * curve - mQuickStopAccumulator;

            if (mQuickStopAccumulator > 1) {
                mQuickStopAccumulator -= 1;
            } else if (mQuickStopAccumulator < -1) {
                mQuickStopAccumulator += 1;
            } else {
                mQuickStopAccumulator = 0.0;
            }
        }

        double leftMotorOutput = throttle + angularPower;
        double rightMotorOutput = throttle - angularPower;

        // If rotation is overpowered, reduce both outputs to within acceptable range
        if (overPower) {
            if (leftMotorOutput > 1.0) {
                rightMotorOutput -= leftMotorOutput - 1.0;
                leftMotorOutput = 1.0;
            } else if (rightMotorOutput > 1.0) {
                leftMotorOutput -= rightMotorOutput - 1.0;
                rightMotorOutput = 1.0;
            } else if (leftMotorOutput < -1.0) {
                rightMotorOutput -= leftMotorOutput + 1.0;
                leftMotorOutput = -1.0;
            } else if (rightMotorOutput < -1.0) {
                leftMotorOutput -= rightMotorOutput + 1.0;
                rightMotorOutput = -1.0;
            }
        }

        // Normalize the wheel speeds
        double maxMagnitude = Math.max(Math.abs(leftMotorOutput), Math.abs(rightMotorOutput));
        if (maxMagnitude > 1.0) {
            leftMotorOutput /= maxMagnitude;
            rightMotorOutput /= maxMagnitude;
        }

        setOpenLoop(new DriveSignal(leftMotorOutput, rightMotorOutput));
    }

    /**
     * the turning algorithm is the same as the one used in DifferentialDrive.
     * @param throttle forward amount
     * @param turn turning amount
     */
    public synchronized void setArcadeDrive(double throttle, double turn) {
        throttle = Util.limit(throttle, -1, 1);
        throttle = Util.deadBand(throttle, 0.04);

        turn = Util.limit(turn, -1, 1);
        turn = Util.deadBand(turn, 0.04);

        throttle = Math.copySign(throttle * throttle, throttle);
        turn = Math.copySign(turn * turn, turn);

        double leftMotorOutput;
        double rightMotorOutput;

        double maxInput = Math.copySign(Math.max(Math.abs(throttle), Math.abs(turn)), throttle);

        if (throttle >= 0.0) {
            if (turn >= 0.0) {
                leftMotorOutput = maxInput;
                rightMotorOutput = throttle - turn;
            } else {
                leftMotorOutput = throttle + turn;
                rightMotorOutput = maxInput;
            }
        } else {
            if (turn >= 0.0) {
                leftMotorOutput = throttle + turn;
                rightMotorOutput = maxInput;
            } else {
                leftMotorOutput = maxInput;
                rightMotorOutput = throttle - turn;
            }
        }

        setOpenLoop(new DriveSignal(Util.limit(leftMotorOutput, -1.0, 1.0), Util.limit(rightMotorOutput, -1.0, 1.0)));
    }

    /**
     * sets velocity demand during path following mode. if not in path following mode, turns on break mode and limits stator current.
     * @param signal the drive signal to set motor velocity
     * @param feedforward the drive signal to set motor feedforward
     */
    public synchronized void setVelocity(DriveSignal signal, DriveSignal feedforward) {
        if (mDriveControlState != DriveControlState.PATH_FOLLOWING) {
            setBrakeMode(true);
            setStatorCurrentLimit(35);
            mDriveControlState = DriveControlState.PATH_FOLLOWING;
        }

        mPeriodicIO.left_demand = signal.getLeft();
        mPeriodicIO.right_demand = signal.getRight();
        mPeriodicIO.left_feedforward = signal.getLeft();
        mPeriodicIO.right_feedforward = signal.getRight();
    }

    /**
     * Initializes the path and sets the drive train into the path following state.
     * @param path path that the robot will follow
     * @param reversed NOT APPLICABLE but reflects the path horizontally
     */
    public synchronized void setDrivePath(Path path, boolean reversed) {
        if (mCurrentPath != path || mDriveControlState != DriveControlState.PATH_FOLLOWING) {
            RobotState.getInstance().resetDistanceDriven();
            mPathFollower = new PathFollower(path, reversed,
                    new PathFollower.Parameters(
                            new LookAhead(Constants.kMinLookAhead, Constants.kMaxLookAhead,
                                    Constants.kMinLookAheadSpeed, Constants.kMaxLookAheadSpeed),
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

    /**
     * returns true if path following is done or doesn't exist and false if not.
     * Used in actions to see when the path is complete to end the action
     * @return if path following exists, check if it's finished
     */
    public synchronized boolean isDoneWithPath() {
        if (mDriveControlState == DriveControlState.PATH_FOLLOWING && mPathFollower != null) {
            return mPathFollower.isFinished();
        }
        TelemetryUtil.print("Robot is not in a path following state", PrintStyle.NONE, false);
        return true;
    }

    /**
     * force finishes path following in case patfollowing needs to be exited for emergency
     */
    public synchronized void forceDoneWithPath() {
        if (mDriveControlState == DriveControlState.PATH_FOLLOWING && mPathFollower != null) {
            mPathFollower.forceFinish();
        }
        TelemetryUtil.print("Robot is not in a path following state", PrintStyle.NONE, false);
    }

    /**
     * continuously updates path following state. Reads from robot_state sets velocity to the motors according to the current path.
     * @param timestamp current timestamp in milliseconds
     */
    private void updatePathFollower(double timestamp) {
        if (mDriveControlState == DriveControlState.PATH_FOLLOWING && mPathFollower != null) {
            RobotState robot_state = RobotState.getInstance();
            Pose2d field_to_vehicle = robot_state.getLatestFieldToVehicle().getValue();
            Twist2d command = mPathFollower.update(timestamp, field_to_vehicle, robot_state.getDistanceDriven(),
                    robot_state.getPredictedVelocity().dx);
            if (!mPathFollower.isFinished()) {
                DriveSignal setpoint = Kinematics.inverseKinematics(command);
                setVelocity(setpoint, new DriveSignal(0, 0));
            } else {
                if (!mPathFollower.isForceFinished()) {
                    setVelocity(new DriveSignal(0, 0), new DriveSignal(0, 0));
                }
            }
        } else {
            TelemetryUtil.print("Drive is not in a path following state", PrintStyle.WARNING, false);
        }
    }

    /**
     * checks if the robot has passed a specific marker
     * @param marker marker for pathfollower to check if it has passed it.
     * @return true if the robot has passed the marker and false if not
     */
    public synchronized boolean hasPassedMarker(String marker) {
        if (mDriveControlState == DriveControlState.PATH_FOLLOWING && mPathFollower != null) {
            return mPathFollower.hasPassedMarker(marker);
        } else {
            TelemetryUtil.print("Drive is not in a path following state", PrintStyle.NONE, false);
            return false;
        }
    }

    /**
     * Gets the heading of the robot (using navx)
     * @return a Rotation2d object representing the current direction of the robot
     */
    public synchronized Rotation2d getHeading() {
        return mPeriodicIO.heading;
    }

    /**
     * Sets the heading variable of periodicIO to the heading passed in.
     * It sets an error value (how many degrees off target) based on the current robot heading and value passed in.
     * @param heading the Rotation2d objec that represents the desired heading direction
     */
    public synchronized void setHeading(Rotation2d heading) {
        mGyroOffset = heading.rotateBy(Rotation2d.fromDegrees(mNavx.getHeading()).inverse());
        mPeriodicIO.heading = heading;
    }

    /**
     * Sets left and right encoder cound to zero.
     */
    public synchronized void resetEncoders() {
        PheonixUtil.checkError(mLeftMaster.setSelectedSensorPosition(0, 0, Constants.kTimeOutMs),
                mLeftMaster.getName() + " failed to reset encoder", true);
        PheonixUtil.checkError(mRightMaster.setSelectedSensorPosition(0, 0, Constants.kTimeOutMs),
                mRightMaster.getName() + " failed to reset encoder", true);
    }

    /**
     * Sets all motors to the specified brake mode. And updates inner break mode state.
     * @param enableBrake if true sets break mode, if false sets to neutral mode.
     */
    public synchronized void setBrakeMode(boolean enableBrake) {
        if (enableBrake != mIsBrakeMode) {
            NeutralMode mode = enableBrake ? NeutralMode.Brake : NeutralMode.Brake;
            mLeftMaster.setNeutralMode(mode);
            mLeftSlave.setNeutralMode(mode);
            mRightMaster.setNeutralMode(mode);
            mRightSlave.setNeutralMode(mode);
            mIsBrakeMode = enableBrake;
        }
    }

    /**
     * Sets all motors' stator current to specified amps
     * @param amps stator limit to set all motors to in amps
     */
    public synchronized void setStatorCurrentLimit(int amps) {
        TalonFXUtil.setStatorCurrentLimit(mLeftMaster, amps);
        TalonFXUtil.setStatorCurrentLimit(mLeftSlave, amps);
        TalonFXUtil.setStatorCurrentLimit(mRightMaster, amps);
        TalonFXUtil.setStatorCurrentLimit(mRightSlave, amps);
    }

    /**
     * checks all talons for sticky faults, and prints errors as needed (done in .checkSensorFaults())
     * if faults are present, they are cleared
     */
    public synchronized void handleFaults() {
        TalonFXUtil.checkSensorFaults(mLeftMaster);
        TalonFXUtil.checkSlaveFaults(mLeftSlave, Ports.DRIVE_LEFT_MASTER_ID);
        TalonFXUtil.checkSensorFaults(mRightMaster);
        TalonFXUtil.checkSlaveFaults(mRightSlave, Ports.DRIVE_RIGHT_MASTER_ID);
    }

    /**
     * checks if any of the motors is above the falcon heat threshold
     * @return true if drive is overheating false if not
     */
    public synchronized boolean isDriveOverheating() {
        return mPeriodicIO.left_master_temperature > Constants.kFalconHeatThreshold
                || mPeriodicIO.left_slave_temperature > Constants.kFalconHeatThreshold
                || mPeriodicIO.right_master_temperature > Constants.kFalconHeatThreshold
                || mPeriodicIO.right_slave_temperature > Constants.kFalconHeatThreshold;
    }

    /**
     * enum to keep track of drive control status
     */
    private enum DriveControlState {
        OPEN_LOOP, PATH_FOLLOWING;
    }

    /**
     * resets heading and encoder
     */
    @Override
    public void zeroSensors() {
        setHeading(Rotation2d.identity());
        resetEncoders();
    }

    /**
     * stops the drivetrain. Sets open loop to neutral
     */
    @Override
    public void stop() {
        setOpenLoop(DriveSignal.NEUTRAL);
    }

    /**
     * Outputs telemetry messages to .dsevents or smartdashboard
     * 
     * Outputs if the drive train is overheating
     */
    @Override
    public void outputTelemetry() {
        SmartDashboard.putBoolean("Is Drive Overheathing", isDriveOverheating());
    }

    

    @Override
    public boolean checkSystem() {

        return false;
    }

}
