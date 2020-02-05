/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.drivers.LazyTalonSRX;
import frc.lib.drivers.MotorChecker;
import frc.lib.drivers.PheonixUtil;
import frc.lib.drivers.TalonSRXChecker;
import frc.lib.drivers.TalonSRXFactory;
import frc.lib.drivers.TalonSRXUtil;
import frc.robot.Constants;
import frc.robot.Ports;
import frc.robot.loops.ILooper;
import frc.robot.loops.Loop;

/**
 * TODO: Need to add ball detection logic. Sensor array or limelight in drivetrain
 */
public class Intake extends Subsystem {
    private static Intake mInstance = null;
    public static Intake getInstance() {
        if(mInstance == null) {
            mInstance = new Intake();
        }
        return mInstance;
    }

    //debug
    private final boolean debug = false;

    //hardware
    private final Solenoid mIntakeSolenoid;
    private final LazyTalonSRX mInnerIntakeMotor, mOuterIntakeMotor;

    //control states
    private IntakeControlState mCurrentState = IntakeControlState.OFF;
    private boolean mStateChanged = false;
    private double mStateChangeTimestamp = 0;


    private void configureIntakeMotor(LazyTalonSRX talon) {
        PheonixUtil.checkError(talon.configVoltageCompSaturation(12.0, Constants.kTimeOutMs),
            talon.getName() + " failed to set voltage compensation", true);
        PheonixUtil.checkError(talon.configVoltageMeasurementFilter(32, Constants.kTimeOutMs),
            talon.getName() + " failed to set voltage meas. filter", true);
        talon.enableVoltageCompensation(true);

        TalonSRXUtil.setCurrentLimit(talon, 25);
        
        talon.setNeutralMode(NeutralMode.Coast);
    }

    private Intake() {
        mIntakeSolenoid = new Solenoid(Ports.PCM_ID, Ports.INTAKE_SOLENOID_PORT);
        
        mInnerIntakeMotor = TalonSRXFactory.createDefaultTalon("Inner Intake Motor", Ports.INTAKE_INNER_MOTOR_ID);
        configureIntakeMotor(mInnerIntakeMotor);

        mOuterIntakeMotor = TalonSRXFactory.createDefaultTalon("Outer Intake Motor", Ports.INTAKE_OUTER_MOTOR_ID);
        configureIntakeMotor(mOuterIntakeMotor);      
    }


    @Override
    public void registerEnabledLoops(ILooper mEnabledLooper) {
        mEnabledLooper.register(new Loop() {

            @Override
            public void onStart(double timestamp) {

            }

            @Override
            public void onLoop(double timestamp) {

            }

            @Override
            public void onStop(double timestamp) {
                stop();

            }

        });
    }


    public enum IntakeControlState {
        OFF(0.0, 0.0, false),
        IDLE_WHILE_DEPLOYED(0.0, 0.0, true),
        STORE(0.3, -0.6, true),
        INTAKE(0.7, 0.7, true),
        OUTAKE(-0.5, -0.5, true);

        public double innerIntakeSpeed = 0.0;
        public double outerIntakeSpeed = 0.0;
        public boolean deployIntake = false;

        private IntakeControlState(double innerIntakeSpeed, double outerIntakeSpeed, boolean deployIntake) {
            this.innerIntakeSpeed = innerIntakeSpeed;
            this.outerIntakeSpeed = outerIntakeSpeed;
            this.deployIntake = deployIntake;
        }
    }


    public synchronized IntakeControlState getState() {
        return mCurrentState;
    }

    public synchronized void setState(IntakeControlState newState) {
        if(newState != mCurrentState) {
            mStateChanged = true;
            mStateChangeTimestamp = Timer.getFPGATimestamp();
        }
        mCurrentState = newState;
    }

    public synchronized void setInnerIntakeSpeed(double speed) {
        mInnerIntakeMotor.set(ControlMode.PercentOutput, speed);
    }

    public synchronized void setOuterIntakeSpeed(double speed) {
        mOuterIntakeMotor.set(ControlMode.PercentOutput, speed);
    }

    public synchronized void setDeployState(boolean isDeployed) {
        mIntakeSolenoid.set(isDeployed);
    }


    public synchronized void conformToState(IntakeControlState desiredState) {
        setState(desiredState);
        setInnerIntakeSpeed(desiredState.innerIntakeSpeed);
        setOuterIntakeSpeed(desiredState.outerIntakeSpeed);
        setDeployState(desiredState.deployIntake);
    }


    @Override
    public void stop() {
        conformToState(IntakeControlState.OFF);
    }

    @Override
    public boolean checkSystem() {
        mIntakeSolenoid.set(true);

        boolean check = TalonSRXChecker.checkMotors(this,
            new ArrayList<MotorChecker.MotorConfig<LazyTalonSRX>>() {    
            private static final long serialVersionUID = 1L;

                {
                    add(new MotorChecker.MotorConfig<>(mOuterIntakeMotor));
                    add(new MotorChecker.MotorConfig<>(mInnerIntakeMotor));
                }
            }, new MotorChecker.CheckerConfig() {
                {
                    mOutputPercent = 0.5;
                    mRuntime = 1;
                    mWaittime = 0.3;
                    mRPMSupplier = null;
                }
            });
        
        mIntakeSolenoid.set(false);

        return check;
        

    }

    @Override
    public void outputTelemetry() {
        if(debug) {
            SmartDashboard.putString("Intake State", mCurrentState.toString());

            SmartDashboard.putBoolean("Intake Solenoid", mIntakeSolenoid.get());

            SmartDashboard.putNumber("Inner Intake Supply Current", mInnerIntakeMotor.getSupplyCurrent());
            SmartDashboard.putNumber("Inner Intake Stator Current", mInnerIntakeMotor.getStatorCurrent());
            SmartDashboard.putNumber("Inner Intake Output", mInnerIntakeMotor.getLastSet());

            SmartDashboard.putNumber("Outer Intake Supply Current", mOuterIntakeMotor.getSupplyCurrent());
            SmartDashboard.putNumber("Outer Intake Stator Current", mOuterIntakeMotor.getStatorCurrent());
            SmartDashboard.putNumber("Outer Intake Output", mOuterIntakeMotor.getLastSet());
        }
    }

}
