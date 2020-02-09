
package frc.robot.subsystems;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.control.LookAhead;
import frc.lib.control.Path;
import frc.lib.control.PathFollower;
import frc.lib.drivers.LazyTalonFX;
import frc.lib.drivers.MotorChecker;
import frc.lib.drivers.PheonixUtil;
import frc.lib.drivers.TalonFXChecker;
import frc.lib.drivers.TalonFXFactory;
import frc.lib.drivers.TalonFXUtil;
import frc.lib.geometry.Pose2d;
import frc.lib.geometry.Rotation2d;
import frc.lib.geometry.Twist2d;
import frc.lib.sensors.Navx;
import frc.lib.util.DriveSignal;
import frc.lib.util.PIDController;
import frc.lib.util.TelemetryUtil;
import frc.lib.util.Util;
import frc.lib.util.TelemetryUtil.PrintStyle;
import frc.robot.Constants;
import frc.robot.Kinematics;
import frc.robot.Ports;
import frc.robot.RobotState;
import frc.robot.loops.ILooper;
import frc.robot.loops.Loop;

public class Drive extends Subsystem {

    private static Drive mInstance = null;

    public static Drive getInstance() {
        if (mInstance == null) {
            mInstance = new Drive();
        }
        return mInstance;
    }

    // hardware
    private LazyTalonFX mLeftMaster, mLeftSlave, mRightMaster, mRightSlave;
    private Navx mNavx;

    // hardware states
    private boolean mIsBrakeMode;
    private Rotation2d mGyroOffset = Rotation2d.identity();
    private PeriodicIO mPeriodicIO;

    // controllers
    private PathFollower mPathFollower;
    private Path mCurrentPath;
    private PIDController mTurnController;
    

    // control states
    private DriveControlState mDriveControlState;


    /**
     * sets the motor's inversion, open loop ramp, closed loop ramp, voltage comp sat, and current limits
     * @param falcon the falcon being set up
     * @param inversion inverted state
     */
    private synchronized void configureMotorForDrive(LazyTalonFX falcon, InvertType inversion) {
        falcon.setInverted(inversion);

        PheonixUtil.checkError(falcon.configVoltageCompSaturation(12.0, Constants.kTimeOutMs),
                falcon.getName() + " failed to set voltage compensation", true);
    
        falcon.enableVoltageCompensation(true);
        
        PheonixUtil.checkError(falcon.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 35, 35, 0)),
                falcon.getName() + " failed to set output current limit", true);

        PheonixUtil.checkError(falcon.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 60, 60, 0)),
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
        PheonixUtil.checkError(falcon.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, Constants.kTimeOutMs),
            falcon.getName() + " failed to set feedback sensor", true);
        
        falcon.setSensorPhase(sensorPhase);

        PheonixUtil.checkError(falcon.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_50Ms, Constants.kTimeOutMs), 
                falcon.getName() + " failed to set velocity meas. period", true);
        
        PheonixUtil.checkError(falcon.configVelocityMeasurementWindow(1, Constants.kTimeOutMs), 
                falcon.getName() + " failed to set velocity meas. window", true);
        
        PheonixUtil.checkError(falcon.configOpenloopRamp(0.3, Constants.kTimeOutMs),
                falcon.getName() + " failed to set open loop ramp rate", true);

        PheonixUtil.checkError(falcon.configClosedloopRamp(0.0, Constants.kTimeOutMs),
                falcon.getName() + " failed to set closed loop ramp rate", true);
        
        PheonixUtil.checkError(falcon.configNeutralDeadband(0.0, Constants.kTimeOutMs), 
                falcon.getName() + " failed to set neutral deadband", true);   
    }

    private Drive() {
        mPeriodicIO = new PeriodicIO();

        mLeftMaster = TalonFXFactory.createDefaultFalcon("Drive Left Master", Ports.DRIVE_LEFT_MASTER_ID);
        configureMasterForDrive(mLeftMaster, InvertType.None, false);

        mLeftSlave = TalonFXFactory.createSlaveFalcon("Drive Left Slave", Ports.DRIVE_LEFT_SLAVE_ID,
             Ports.DRIVE_LEFT_MASTER_ID);
        configureMotorForDrive(mLeftSlave, InvertType.FollowMaster);
        mLeftSlave.setMaster(mLeftMaster);

        mRightMaster = TalonFXFactory.createDefaultFalcon("Drive Right Master", Ports.DRIVE_RIGHT_MASTER_ID);
        configureMasterForDrive(mRightMaster, InvertType.InvertMotorOutput, false);

        mRightSlave = TalonFXFactory.createSlaveFalcon("Drive Right Slave", Ports.DRIVE_RIGHT_SLAVE_ID,
             Ports.DRIVE_RIGHT_MASTER_ID);
        configureMotorForDrive(mRightSlave, InvertType.FollowMaster);
        mRightSlave.setMaster(mRightMaster);

        mNavx = Navx.getInstance();

        mIsBrakeMode = false;
        setBrakeMode(true);
        setOpenLoop(DriveSignal.NEUTRAL);

        /*SmartDashboard.putNumber("Turn kP", 0.01);
        SmartDashboard.putNumber("Turn kI", 0.0);
        SmartDashboard.putNumber("Turn kD", 0.0);

        mTurnController = new PIDController(SmartDashboard.getNumber("Turn kP", 0),
             SmartDashboard.getNumber("Turn kI", 0.0), SmartDashboard.getNumber("Turn kD", 0.0),
             () -> getHeading().getDegrees(), 5);*/

        //mTurnController.setIRange(5);
        //mTurnController.enableDebug();

        
    }

    /**
     * tracks the inputs and outputs of the drive train
     */
    private static class PeriodicIO {
        // inputs
        public double timestamp;
        public double left_position;
        public double right_position;
        public double left_distance;
        public double right_distance;
        public double left_velocity_per_50ms;
        public double right_velocity_per_50ms;
        public Rotation2d heading = Rotation2d.identity();
        public Pose2d error = Pose2d.identity();

        public double left_master_temperature;
        public double left_slave_temperature;
        public double right_master_temperature;
        public double right_slave_temperature;

        // outputs
        public double left_demand;
        public double right_demand;
    }

    /**
     * reads encoder position, change in position, velocity, heading, and temperature
     */
    @Override
    public synchronized void readPeriodicInputs() {
        mPeriodicIO.timestamp = Timer.getFPGATimestamp();

        double prevLeftPosition = mPeriodicIO.left_position;
        double prevRightPosition = mPeriodicIO.right_position;

        mPeriodicIO.left_position = mLeftMaster.getSelectedSensorPosition();
        mPeriodicIO.right_position = mRightMaster.getSelectedSensorPosition();

        double deltaLeftPosition = mPeriodicIO.left_position - prevLeftPosition;
        double deltaRightPosition = mPeriodicIO.right_position - prevRightPosition;

        mPeriodicIO.left_distance += deltaLeftPosition;
        mPeriodicIO.right_distance += deltaRightPosition;

        mPeriodicIO.left_velocity_per_50ms = mLeftMaster.getSelectedSensorVelocity();
        mPeriodicIO.right_velocity_per_50ms = mRightMaster.getSelectedSensorVelocity();

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
        if (mDriveControlState != null) {
            mLeftMaster.set(ControlMode.PercentOutput, mPeriodicIO.left_demand);
            mRightMaster.set(ControlMode.PercentOutput, mPeriodicIO.right_demand);
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
                    //handleFaults();
                    //System.out.println(mDriveControlState);
                    switch (mDriveControlState) {
                    case OPEN_LOOP:
                        break;
                    case PATH_FOLLOWING:
                        if (mPathFollower != null) {
                            updatePathFollower(timestamp);
                        }
                        break;
                    case TURN_PID:
                        updateTurnController();
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
                setBrakeMode(true);
            }
        });
    }

    /**
     * @return raw position of left encoder
     */
    public double getLeftEncoderPosition() {
        return (mPeriodicIO.left_position / 2048) * 0.0972 * Math.PI * Constants.kDriveWheelDiameter;
    }

    /**
     * 
     * @return raw position of right encoder
     */
    public double getRightEncoderPosition() {
        return (mPeriodicIO.right_position / 2048) * 0.0972 * Math.PI * Constants.kDriveWheelDiameter;
    }

    /**
     * @return left encoder distance (raw) per second
     */
    public double getLeftLinearVelocity() {
        return (mPeriodicIO.left_velocity_per_50ms / 2048) * 0.0972 * Math.PI * Constants.kDriveWheelDiameter * 20.0;
    }

    /**
     * 
     * @return right encoder distance (raw) per second
     */
    public double getRightLinearVelocity() {
        return (mPeriodicIO.right_velocity_per_50ms / 2048) * 0.0972 * Math.PI * Constants.kDriveWheelDiameter * 20.0;
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
            PheonixUtil.checkError(mLeftMaster.configNeutralDeadband(0.04, Constants.kTimeOutMs),
                 mLeftMaster.getName() + " failed to set neutral deadband on openloop transition", true);
            PheonixUtil.checkError(mRightMaster.configNeutralDeadband(0.04, Constants.kTimeOutMs),
                 mRightMaster.getName() + " failed to set neutral deadband on openloop transition", true);
            mDriveControlState = DriveControlState.OPEN_LOOP;
            //System.out.println("set open loop in setopenloop in das tingso");
        }

        mPeriodicIO.left_demand = signal.getLeft();
        mPeriodicIO.right_demand = signal.getRight();
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
        //System.out.println("arcade drive");
        setOpenLoop(new DriveSignal(Util.limit(leftMotorOutput, -1.0, 1.0), Util.limit(rightMotorOutput, -1.0, 1.0)));
        
    }


    public synchronized void setTurnPIDTarget(Rotation2d heading) {
        if(mDriveControlState != DriveControlState.TURN_PID) {
            setBrakeMode(true);
            setStatorCurrentLimit(35);
            PheonixUtil.checkError(mLeftMaster.configNeutralDeadband(0.0, Constants.kTimeOutMs),
                 mLeftMaster.getName() + " failed to set neutral deadband on turn pid transition", true);
            PheonixUtil.checkError(mRightMaster.configNeutralDeadband(0.0, Constants.kTimeOutMs),
                 mRightMaster.getName() + " failed to set neutral deadband on turn pid transition", true);
            
            mTurnController.reset();
            mDriveControlState = DriveControlState.TURN_PID;
        }
    
        mTurnController.setConstants(SmartDashboard.getNumber("Turn kP", 0),
            SmartDashboard.getNumber("Turn kI", 0.0), SmartDashboard.getNumber("Turn kD", 0.0));
        
        mTurnController.setDesiredValue(heading.getDegrees());
    }


    private synchronized void updateTurnController() {
        if(mDriveControlState == DriveControlState.TURN_PID) {
            double output = mTurnController.getOutput();
            mPeriodicIO.left_demand = output;
            mPeriodicIO.right_demand = -output;
        } else {
            TelemetryUtil.print("Robot is not in a PID turning state", PrintStyle.ERROR, true);
        }
    }

    /**
     * sets velocity demand during path following mode. if not in path following mode, turns on break mode and limits stator current.
     * @param signal the drive signal to set motor velocity
     * @param feedforward the drive signal to set motor feedforward
     */
    public synchronized void setVelocity(DriveSignal signal) {
        if (mDriveControlState != DriveControlState.PATH_FOLLOWING) {
            setBrakeMode(true);
            setStatorCurrentLimit(35);
            PheonixUtil.checkError(mLeftMaster.configNeutralDeadband(0.0, Constants.kTimeOutMs),
                 mLeftMaster.getName() + " failed to set neutral deadband on pathing transition", true);
            PheonixUtil.checkError(mRightMaster.configNeutralDeadband(0.0, Constants.kTimeOutMs),
                 mRightMaster.getName() + " failed to set neutral deadband on pathing transition", true);
            mDriveControlState = DriveControlState.PATH_FOLLOWING;
        }

        mPeriodicIO.left_demand = signal.getLeft();
        mPeriodicIO.right_demand = signal.getRight();
    }

    /**
     * Initializes the path and sets the drive train into the path following state.
     * @param path path that the robot will follow
     * @param reversed NOT APPLICABLE but reflects the path horizontally
     */
    public synchronized void setDrivePath(Path path, boolean reversed) {
        if (mCurrentPath != path || mDriveControlState != DriveControlState.PATH_FOLLOWING) {
            RobotState.getInstance().resetDistanceDriven();
            mDriveControlState = DriveControlState.PATH_FOLLOWING;
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
            
            mCurrentPath = path;
            System.out.println("Path SET!!!!!!!!!!!!!!!!!!");
        } else {
            setVelocity(DriveSignal.BRAKE);
            //System.out.println("WE ARE FAILURS!!!!!!!!!!");
        }
    }

    /**
     * returns true if path following is done or doesn't exist and false if not.
     * Used in actions to see when the path is complete to end the action
     * @return if path following exists, check if it's finished
     */
    public synchronized boolean isDoneWithPath() {
        //System.out.println("state: " + mDriveControlState.toString());
        //System.out.println(" mPathFollower: " + (mPathFollower!=null));
        if (mDriveControlState == DriveControlState.PATH_FOLLOWING && mPathFollower != null) {
            return mPathFollower.isFinished();
        } else {
            TelemetryUtil.print("Robot is not in a path following statee", PrintStyle.NONE, false);
            return true;
        }
        
    }

    /**
     * force finishes path following in case patfollowing needs to be exited for emergency
     */
    public synchronized void forceDoneWithPath() {
        if (mDriveControlState == DriveControlState.PATH_FOLLOWING && mPathFollower != null) {
            mPathFollower.forceFinish();
        }
        TelemetryUtil.print("Robot is not in a path following state!", PrintStyle.NONE, false);
    }

    /**
     * continuously updates path following state. Reads from robot_state sets velocity to the motors according to the current path.
     * @param timestamp current timestamp in milliseconds
     */
    private synchronized void updatePathFollower(double timestamp) {
        if (mPathFollower != null) {
            RobotState robot_state = RobotState.getInstance();
            Pose2d field_to_vehicle = robot_state.getLatestFieldToVehicle().getValue();
            Twist2d command = mPathFollower.update(timestamp, field_to_vehicle, robot_state.getDistanceDriven(),
                    robot_state.getPredictedVelocity().dx);



            if (!mPathFollower.isFinished()) {
                DriveSignal setpoint = Kinematics.inverseKinematics(command);
                SmartDashboard.putNumber("Left Output", setpoint.getLeft());
                SmartDashboard.putNumber("Right Output", setpoint.getRight());
                //System.out.println("Output: " + setpoint);
                setVelocity(setpoint);
            } else {
                if (!mPathFollower.isForceFinished()) {
                    setVelocity(new DriveSignal(0, 0));
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
            TelemetryUtil.print("Is Brake Mode: " + enableBrake, PrintStyle.ERROR, true);
            NeutralMode mode = enableBrake ? NeutralMode.Brake : NeutralMode.Coast;
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
        OPEN_LOOP, PATH_FOLLOWING, TURN_PID;
    }

    /**
     * resets heading and encoder
     */
    @Override
    public void zeroSensors() {
        mPeriodicIO = new PeriodicIO();
        mNavx.reset();
        setHeading(Rotation2d.identity());
        resetEncoders();
    }

    /**
     * stops the drivetrain. Sets open loop to neutral
     */
    @Override
    public void stop() {
       // System.out.println("drive stop called");
        setVelocity(DriveSignal.NEUTRAL);
    }

    /**
     * Outputs telemetry messages to .dsevents or smartdashboard
     * 
     * Outputs if the drive train is overheating
     */
    @Override
    public void outputTelemetry() {
        SmartDashboard.putBoolean("Is Drive Overheathing", isDriveOverheating());
        SmartDashboard.putNumber("Left Drive Velocity", getLeftLinearVelocity());
        SmartDashboard.putNumber("Right Drive Velocity", getRightLinearVelocity());
        SmartDashboard.putNumber("Left Position", getLeftEncoderPosition());
        SmartDashboard.putNumber("Right Position", getRightEncoderPosition());
        SmartDashboard.putNumber(" NavX Heading", mNavx.getHeading()); 
    }

    

    @Override
    public boolean checkSystem() {

        return false;
    }


    public void testDriveConfiguration() {

        TalonFXChecker.testMotors(this,
            new ArrayList<MotorChecker.MotorConfig<LazyTalonFX>>() {    
            private static final long serialVersionUID = 1L;

                    {
                    add(new MotorChecker.MotorConfig<>(mLeftMaster));
                    add(new MotorChecker.MotorConfig<>(mLeftSlave));
                }
            }, new MotorChecker.TesterConfig() {
                {
                    mOutputPercent = 0.3;
                    mRPMSupplier = () -> mLeftMaster.getSelectedSensorVelocity(0);
                }
            });


        TalonFXChecker.testMotors(this,
            new ArrayList<MotorChecker.MotorConfig<LazyTalonFX>>() {
            private static final long serialVersionUID = 1L;

                    {
                    add(new MotorChecker.MotorConfig<>(mRightMaster));
                    add(new MotorChecker.MotorConfig<>(mRightSlave));
                }
            }, new MotorChecker.TesterConfig() {
                {
                    mRPMSupplier = () -> mRightMaster.getSelectedSensorVelocity(0);
                    mOutputPercent = 0.3;
                }
            });

    }

}