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

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.drivers.LazySparkMax;
import frc.lib.drivers.SparkMaxFactory;
import frc.lib.drivers.SparkMaxUtil;
import frc.lib.sensors.ColorSensor;
import frc.robot.Constants;
import frc.robot.Ports;
import frc.robot.loops.ILooper;
import frc.robot.loops.Loop;

/*
 * Add your docs here.
 */
public class Spinner extends Subsystem {

    private static Spinner mInstance = null;

    public static Spinner getInstance() {
        if(mInstance == null) {
            mInstance = new Spinner();
        }
        return mInstance;
    }

    //debug
    private final boolean debug = false;

    //hardware
    private final LazySparkMax mSpinnerMotor;
    private final ColorSensor mColorSensor;
    private final Solenoid mSpinnerSolenoid;
    private final CANEncoder mEncoder;

    //controllers
    private final CANPIDController mPidController;

    //control states
    private SpinnerControlState mCurrentState;
    private boolean mStateChanged = false;
    private double mStateChangeTimestamp = 0;


    private void configureSparkForSpinner(LazySparkMax spark) {
        SparkMaxUtil.checkError(spark.enableVoltageCompensation(12.0), spark.getName() + " failed to set voltage comp.", true);
        SparkMaxUtil.checkError(spark.setSmartCurrentLimit(20), spark.getName() + " failed to set current limit", true);
        SparkMaxUtil.checkError(spark.setOpenLoopRampRate(0.0), spark.getName() + " failed to set open loop ramp", true);
        SparkMaxUtil.checkError(spark.setClosedLoopRampRate(0.4), spark.getName() + " failed to set closed loop ramp", true);
    }

    private void setPIDConstants() {
        SparkMaxUtil.checkError(mPidController.setP(0.0), mSpinnerMotor.getName() + " failed to set kP", true);
        SparkMaxUtil.checkError(mPidController.setI(0.0), mSpinnerMotor.getName() + " failed to set kI", true);
        SparkMaxUtil.checkError(mPidController.setD(0.0), mSpinnerMotor.getName() + " failed to set kD", true);
        SparkMaxUtil.checkError(mPidController.setFF(0.0), mSpinnerMotor.getName() + " failed to set kF", true);
        SparkMaxUtil.checkError(mPidController.setOutputRange(-1.0, 1.0), mSpinnerMotor.getName() + " failed to set PID output ranges", true);
    }

    private Spinner() {
        mSpinnerMotor = SparkMaxFactory.createDefaultSparkMax("Spinner Motor", Ports.SPINNER_MOTOR_ID, false);
        configureSparkForSpinner(mSpinnerMotor);

        mEncoder = mSpinnerMotor.getEncoder();

        mPidController = mSpinnerMotor.getPIDController();
        setPIDConstants();

        mSpinnerSolenoid = new Solenoid(Ports.PCM_ID, Ports.SPINNER_SOLENOID_ID);

        mColorSensor = ColorSensor.getInstance();
    }


    @Override
    public void registerEnabledLoops(ILooper mEnabledLooper) {
        mEnabledLooper.register(new Loop() {

            @Override
            public void onStart(double timestamp) {

            }

            @Override
            public void onLoop(double timestamp) {
                switch(mCurrentState) {
                    case OFF:
                        break;
                    case OPEN_LOOP:
                        break;
                    case ROTATION_CONTROL:
                        updateRotationControl();
                        break;
                    case POSITION_CONTROL:
                        updatePositionControl();
                        break;
                }
            }

            @Override
            public void onStop(double timestamp) {
                stop();

            }

        });
    }


    public enum SpinnerControlState {
        OFF,
        OPEN_LOOP,
        ROTATION_CONTROL,
        POSITION_CONTROL
    }


    public synchronized SpinnerControlState getState() {
        return mCurrentState;
    }

    public synchronized void setState(SpinnerControlState newState) {
        if(newState != mCurrentState) {
            mStateChanged = true;
            mStateChangeTimestamp = Timer.getFPGATimestamp();
        }
        mCurrentState = newState;
    }

    public synchronized void setOpenLoop(double percentOutput) {
        if(mCurrentState != SpinnerControlState.OPEN_LOOP) {
            mSpinnerSolenoid.set(true);
            setState(SpinnerControlState.OPEN_LOOP);
        }
        mSpinnerMotor.set(ControlType.kDutyCycle, percentOutput);
    }

    public synchronized void setOff() {
        if(mCurrentState != SpinnerControlState.OFF) {
            mSpinnerMotor.set(ControlType.kDutyCycle, 0.0);
            mSpinnerSolenoid.set(false);
            setState(SpinnerControlState.OFF);
        }
    }

    public synchronized void setRotationControl() {
        if(mCurrentState != SpinnerControlState.ROTATION_CONTROL) {
            mSpinnerSolenoid.set(true);
            mSpinnerMotor.set(ControlType.kPosition, Constants.kCountsPerControlPanelRotation * 3.1);
            setState(SpinnerControlState.ROTATION_CONTROL);
        }

    }

    private void updateRotationControl() {

    }

    public synchronized void setPositionControl() {

    }

    private void updatePositionControl() {

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
