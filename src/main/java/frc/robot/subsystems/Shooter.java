/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.ControlType;
import com.revrobotics.EncoderType;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.drivers.LazySparkMax;
import frc.lib.drivers.LazyTalonFX;
import frc.lib.drivers.LazyTalonSRX;
import frc.lib.drivers.SparkMaxFactory;
import frc.lib.drivers.SparkMaxUtil;
import frc.lib.drivers.TalonFXFactory;
import frc.lib.drivers.TalonSRXFactory;
import frc.lib.util.CircularBuffer;
import frc.robot.Constants;
import frc.robot.Ports;
import frc.robot.loops.ILooper;
import frc.robot.loops.Loop;

/**
 * Add your docs here.
 */
public class Shooter extends Subsystem {

    private static Shooter mInstance = null;

    public static Shooter getInstance() {
        if(mInstance == null) {
            mInstance = new Shooter();
        }
        return mInstance;
    }

    //hardware
    private final LazySparkMax mMasterShooter, mSlaveShooter;
    private final CANEncoder mShooterEncoder;
    //private final LazyTalonSRX mHoodMotor, mIndexer;
    private final Solenoid mRampSolenoid;

    //controllers
    private final CANPIDController mPIDFController;
    private int mCurrentSlot;
    private final int kSpinUpSlot = 0;
    private final int kHoldSlot = 1;

    //states
    private ShooterControlState mControlState;
    private CircularBuffer mKfEstimator = new CircularBuffer(Constants.kShooterKfBufferSize);
    private boolean mOnTarget = false;
    private double mOnTargetStartTime = Double.POSITIVE_INFINITY;
    private PeriodicIO mPeriodicIO;


    private void configSparkForShooter(LazySparkMax sparkMax) {
        SparkMaxUtil.setCurrentLimit(sparkMax, 30, 40);
        SparkMaxUtil.setVoltageCompensation(sparkMax, 12.0);
    }

    private void configEncoderForShooter(CANEncoder encoder, boolean sensorPhase) {
        SparkMaxUtil.checkError(encoder.setInverted(sensorPhase), "shooter encoder " + 
            "failed to set sensor phase", true);

        SparkMaxUtil.checkError(encoder.setMeasurementPeriod(10), "shooter encoder " + 
            "failed to set measurement period", true);

        SparkMaxUtil.checkError(encoder.setPositionConversionFactor(Constants.kGearReduction / 
            Constants.kNeoPPR), "shooter encoder " + "failed to set position conversion", true);
        
        SparkMaxUtil.checkError(encoder.setVelocityConversionFactor((Constants.kGearReduction / Constants.kNeoPPR) * 6000), "shooter encoder " + 
            "failed to set sensor phase", true);
    }


    private void setControllerConstants() {
        mPIDFController.setP(Constants.kShooterkP, kSpinUpSlot);
        mPIDFController.setI(Constants.kShooterkI, kSpinUpSlot);
        mPIDFController.setD(Constants.kShooterkD, kSpinUpSlot);
        mPIDFController.setFF(Constants.kShooterkF, kSpinUpSlot);
        mPIDFController.setIZone(Constants.kShooterIZone, kSpinUpSlot);

        mPIDFController.setP(0.0, kHoldSlot);
        mPIDFController.setI(0.0, kHoldSlot);
        mPIDFController.setD(0.0, kHoldSlot);
        mPIDFController.setFF(Constants.kShooterkF, kHoldSlot);
        mPIDFController.setIZone(0.0, kHoldSlot);

        mPIDFController.setOutputRange(-1.0, 1.0);
    }


    private Shooter() {
        mPeriodicIO = new PeriodicIO();

        mMasterShooter = SparkMaxFactory.createDefaultSparkMax("Left Shooter Neo", Ports.SHOOTER_LEFT_SHOOT_ID, false);
        configSparkForShooter(mMasterShooter);
        
        mSlaveShooter = SparkMaxFactory.createSlaveSparkMax("Right Shooter Neo", Ports.SHOOTER_RIGHT_SHOOT_ID,
             mMasterShooter, true);
        configSparkForShooter(mSlaveShooter);


        mShooterEncoder = mMasterShooter.getEncoder();
        configEncoderForShooter(mShooterEncoder, false);

        mPIDFController = mMasterShooter.getPIDController();
        setControllerConstants();

        mMasterShooter.burnFlash();
        mSlaveShooter.burnFlash();
    

        mRampSolenoid = new Solenoid(Ports.SHOOTER_RAMP_SOLENOID_PORT);
    }


    private static class PeriodicIO {
        //inputs
        public double velocity;
        public double prev_velocity;
        public double velocity_in_ticks_per_100ms;
        public double voltage;
        public double master_shooter_temp;
        public double slave_shooter_temp;

        //outputs
        public double setpoint_rpm;
        public double prev_setpoint_rpm;
    }

    @Override
    public void readPeriodicInputs() {
        mPeriodicIO.velocity = mShooterEncoder.getVelocity();
        mPeriodicIO.velocity_in_ticks_per_100ms = 42.0 / 600.0 * mPeriodicIO.velocity;
        mPeriodicIO.voltage = mMasterShooter.getAppliedOutput() * mMasterShooter.getBusVoltage();

        mPeriodicIO.master_shooter_temp = mMasterShooter.getMotorTemperature();
        mPeriodicIO.slave_shooter_temp = mSlaveShooter.getMotorTemperature();
    }



    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(new Loop() {

            @Override
            public void onStart(double timestamp) {
                synchronized(Shooter.this) {
                    mControlState = ShooterControlState.OPEN_LOOP;
                    mKfEstimator.clear();
                    mOnTarget = false;
                    mOnTargetStartTime = Double.POSITIVE_INFINITY;
                }
            }

            @Override
            public void onLoop(double timestamp) {
                synchronized(Shooter.this) {
                    if(mControlState != ShooterControlState.OPEN_LOOP) {
                        handleClosedLoop(timestamp);
                    } else {
                        mKfEstimator.clear();
                        mOnTarget = false;
                        mOnTargetStartTime = Double.POSITIVE_INFINITY;
                    }
                }
            }

            @Override
            public void onStop(double timestamp) {

            }
            
        });
    }

    public enum ShooterControlState {
        OPEN_LOOP, //for testing purposes
        SPIN_UP, //PIDF controller to reach desired RPM
        HOLD_WHEN_READY, //brief state of calcuating kF required in hold stage
        HOLD //state for mantaining rpm and quickly fixing any error when rapidly firing
    }

    public synchronized void setOpenLoop(double percentOutput) {
        if(mControlState != ShooterControlState.OPEN_LOOP) {
            mControlState = ShooterControlState.OPEN_LOOP;
            SparkMaxUtil.setCurrentLimit(mMasterShooter, 30, 40);
            SparkMaxUtil.setVoltageCompensation(mMasterShooter, 12.0);
            SparkMaxUtil.setCurrentLimit(mSlaveShooter, 30, 40);
            SparkMaxUtil.setVoltageCompensation(mSlaveShooter, 12.0);

        }
     
        mMasterShooter.set(ControlType.kDutyCycle, percentOutput);
    }


    public synchronized void setSpinUp(double setpointRpm) {
        if(mControlState != ShooterControlState.SPIN_UP) {
            configForSpinUp();
        }
        mPeriodicIO.setpoint_rpm = setpointRpm;
    }

    
    public synchronized void setHoldWhenReady(double setpointRpm) {
        if(mControlState == ShooterControlState.SPIN_UP || mControlState == ShooterControlState.OPEN_LOOP) {
            configForHoldWhenReady();
        }
        mPeriodicIO.setpoint_rpm = setpointRpm;
    }



    private void configForSpinUp() {
        mControlState = ShooterControlState.SPIN_UP;
        mCurrentSlot = kSpinUpSlot;

        mMasterShooter.setClosedLoopRampRate(Constants.kShooterRampRate);

        SparkMaxUtil.disableVoltageCompensation(mMasterShooter);
        SparkMaxUtil.disableVoltageCompensation(mSlaveShooter);
    }


    private void configForHoldWhenReady() {
        mControlState = ShooterControlState.HOLD_WHEN_READY;
        mCurrentSlot = kSpinUpSlot;

        mMasterShooter.setClosedLoopRampRate(Constants.kShooterRampRate);

        SparkMaxUtil.disableVoltageCompensation(mMasterShooter);
        SparkMaxUtil.disableVoltageCompensation(mSlaveShooter);
    }

    private void configForHold() {
        mControlState = ShooterControlState.HOLD;
        mCurrentSlot = kHoldSlot;
        mPIDFController.setFF(mKfEstimator.getAverage(), mCurrentSlot);
        mMasterShooter.setClosedLoopRampRate(Constants.kShooterRampRate);

        SparkMaxUtil.setVoltageCompensation(mMasterShooter, 12.0);
        SparkMaxUtil.setVoltageCompensation(mSlaveShooter, 12.0);
    }

    private void resetHold() {
        mKfEstimator.clear();
        mOnTarget = false;
    }

    private double estimateKf(double rpm, double voltage) {
        final double output = 1023.0 / 12.0 * voltage;
        return output/mPeriodicIO.velocity_in_ticks_per_100ms;
    }

    /**
     * This method is the main controller of progressing the shooter through all the
     * stages (spin up -> hold when ready -> hold)
     * When the shooter is in the hold state, it is ready to fire
     */
    private void handleClosedLoop(double timestamp) {

        if(mControlState == ShooterControlState.SPIN_UP) {
            mMasterShooter.set(ControlType.kVelocity, mPeriodicIO.setpoint_rpm, mCurrentSlot);
            resetHold();
        } else if(mControlState == ShooterControlState.HOLD_WHEN_READY) {
            final double abs_error = Math.abs(mPeriodicIO.velocity - mPeriodicIO.setpoint_rpm);
            final boolean on_target_now = mOnTarget ? abs_error < Constants.kShooterStopOnTargetRpm :
                abs_error < Constants.kShooterStartOnTargetRpm;
            
            if(on_target_now && !mOnTarget) {
                mOnTargetStartTime = timestamp;
                mOnTarget = true;
            } else if(!on_target_now) {
                resetHold();
            }

            if(mOnTarget) {
                mKfEstimator.addValue(estimateKf(mPeriodicIO.velocity, mPeriodicIO.voltage));
            }

            if(mKfEstimator.getNumValues() >= Constants.kShooterMinOnTargetSamples) {
                configForHold();
            } else {
                mMasterShooter.set(ControlType.kVelocity, mPeriodicIO.setpoint_rpm, mCurrentSlot);
            }
        }

        if(mControlState == ShooterControlState.HOLD) {
            if(mPeriodicIO.velocity > mPeriodicIO.setpoint_rpm) {
                mKfEstimator.addValue(estimateKf(mPeriodicIO.velocity, mPeriodicIO.voltage));
                mPIDFController.setFF(mKfEstimator.getAverage(), kHoldSlot);
            }
        }
    }

    public synchronized boolean isOnTarget() {
        return mControlState == ShooterControlState.HOLD;
    }

    public synchronized double getSetpointRpm() {
        return mPeriodicIO.setpoint_rpm;
    }



    @Override
    public void stop() {
        setOpenLoop(0.0);
        mPeriodicIO.setpoint_rpm = 0.0;
    }

    @Override
    public boolean checkSystem() {
        return false;
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putBoolean("Ready to Fire?", isOnTarget());
        SmartDashboard.putNumber("Shooter Setpoint RPM", mPeriodicIO.setpoint_rpm);
        SmartDashboard.putNumber("Shooter RPM", mPeriodicIO.velocity);
        
    }
}
