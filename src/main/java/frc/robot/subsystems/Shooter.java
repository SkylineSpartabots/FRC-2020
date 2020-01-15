/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.EncoderType;

import edu.wpi.first.wpilibj.Solenoid;
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
        SparkMaxUtil.setCurrentLimit(sparkMax, 35, 40);
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

    public enum ShooterControlState {
        OPEN_LOOP, //for testing purposes
        SPIN_UP, //PIDF controller to reach desired RPM
        HOLD_WHEN_READY, //brief state of calcuating kF required in hold stage
        HOLD //state for mantaining rpm and quickly fixing any error when rapidly firing
    }

    public synchronized void setOpenLoop(double percentOutput) {
        if(mControlState != ShooterControlState.OPEN_LOOP) {
            mControlState = ShooterControlState.OPEN_LOOP;

        }
    }


    public synchronized void setSpinUp(double setpointRpm) {
        if(mControlState != ShooterControlState.SPIN_UP) {
            configForSpinUp();
        }
       
    }

    
    public synchronized void setHoldWhenReady(double setpointRPM) {
        if(mControlState == ShooterControlState.SPIN_UP || mControlState == ShooterControlState.OPEN_LOOP) {
            configForHoldWhenReady();
        }
        
    }



    private void configForSpinUp() {
        mControlState = ShooterControlState.SPIN_UP;
        mMasterShooter.setClosedLoopRampRate(Constants.kShooterRampRate);

        SparkMaxUtil.disableVoltageCompensation(mMasterShooter);
        SparkMaxUtil.disableVoltageCompensation(mSlaveShooter);

        mCurrentSlot = kSpinUpSlot;


    }


    private void configForHoldWhenReady() {

    }

    private void handleClosedLoop() {

    }



    @Override
    public void stop() {

    }

    @Override
    public boolean checkSystem() {
        return false;
    }

    @Override
    public void outputTelemetry() {

    }
}
