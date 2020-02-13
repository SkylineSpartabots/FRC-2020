/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;

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
import frc.robot.subsystems.requests.Request;


public class Hopper extends Subsystem {

    private static Hopper mInstance = null;
    
    public static Hopper getInstance() {
        if(mInstance == null) {
            mInstance = new Hopper();
        }
        return mInstance;
    }

    //debug
    private final boolean debug = false;

    //hardware
    private final LazyTalonSRX mIndexMotor, mLeftBelt, mRightBelt;


    //control states
    private HopperControlState mCurrentState = HopperControlState.OFF;
    private boolean mStateChanged = false;
    private double mStateChangeTimestamp = 0.0;

    private void configureBeltMotor(LazyTalonSRX talon, InvertType inversion) {
        talon.setInverted(inversion);
        
        PheonixUtil.checkError(talon.configVoltageCompSaturation(12.0, Constants.kTimeOutMs),
            talon.getName() + " failed to set voltage compensation", true);
        PheonixUtil.checkError(talon.configVoltageMeasurementFilter(32, Constants.kTimeOutMs),
            talon.getName() + " failed to set voltage meas. filter", true);
        talon.enableVoltageCompensation(true);

        TalonSRXUtil.setCurrentLimit(talon, 25);
        
        talon.setNeutralMode(NeutralMode.Coast);
    }

    private void configureIndexMotor(LazyTalonSRX talon, InvertType inversion) {
        talon.setInverted(inversion);
        
        PheonixUtil.checkError(talon.configVoltageCompSaturation(12.0, Constants.kTimeOutMs),
            talon.getName() + " failed to set voltage compensation", true);
        PheonixUtil.checkError(talon.configVoltageMeasurementFilter(32, Constants.kTimeOutMs),
            talon.getName() + " failed to set voltage meas. filter", true);
        talon.enableVoltageCompensation(true);

        TalonSRXUtil.setCurrentLimit(talon, 25);
        
        talon.setNeutralMode(NeutralMode.Brake);
    }
    


    private Hopper() {
        mIndexMotor = TalonSRXFactory.createDefaultTalon("Index Motor", Ports.HOPPER_INDEX_ID);
        configureIndexMotor(mIndexMotor, InvertType.None);

        mLeftBelt = TalonSRXFactory.createDefaultTalon("Left Belt Motor", Ports.HOPPER_LEFT_BELT);
        configureBeltMotor(mLeftBelt, InvertType.None);

        mRightBelt = TalonSRXFactory.createDefaultTalon("Right Belt Motor", Ports.HOPPER_RIGHT_BELT);
        configureBeltMotor(mRightBelt, InvertType.InvertMotorOutput);
    }


    public enum HopperControlState {
        OFF(0.0, 0.0, 0.0),
        INDEX(0.9, 0.6, 0.6),
        SLOW_INDEX(0.4, 0.6, 0.6),
        REVERSE(-0.3, -0.5, -0.5);

        public double indexSpeed = 0.0;
        public double leftBeltSpeed = 0.0;
        public double rightBeltSpeed = 0.0;

        private HopperControlState(double indexSpeed, double leftBeltSpeed, double rightBeltSpeed) {
            this.indexSpeed = indexSpeed;
            this.leftBeltSpeed = leftBeltSpeed;
            this.rightBeltSpeed = rightBeltSpeed;
        }
    }


    public synchronized HopperControlState getState() {
        return mCurrentState;
    }

    public synchronized void setState(HopperControlState newState) {
        if(newState != mCurrentState) {
            mStateChanged = true;
            mStateChangeTimestamp = Timer.getFPGATimestamp();
        }
        mCurrentState = newState;
    }

    public synchronized void setBeltSpeed(double leftBeltSpeed, double rightBeltSpeed) {
        mLeftBelt.set(ControlMode.PercentOutput, leftBeltSpeed);
        mRightBelt.set(ControlMode.PercentOutput, rightBeltSpeed);
    }

    public synchronized void setIndexSpeed(double indexSpeed) {
        mIndexMotor.set(ControlMode.PercentOutput, indexSpeed);
    }

    public synchronized void conformToState(HopperControlState desiredState) {
        setState(desiredState);
        setBeltSpeed(desiredState.leftBeltSpeed, desiredState.rightBeltSpeed);
        setIndexSpeed(desiredState.indexSpeed);
    }


    public Request stateRequest(HopperControlState desiredState) {
        return new Request(){
        
            @Override
            public void act() {
                conformToState(desiredState);
            }
        };
    }

    
    @Override
    public void stop() {
        conformToState(HopperControlState.OFF);
    }

    @Override
    public boolean checkSystem() {
        return TalonSRXChecker.checkMotors(this,
            new ArrayList<MotorChecker.MotorConfig<LazyTalonSRX>>() {    
            private static final long serialVersionUID = 1L;

                {
                    add(new MotorChecker.MotorConfig<>(mIndexMotor));
                    add(new MotorChecker.MotorConfig<>(mLeftBelt));
                    add(new MotorChecker.MotorConfig<>(mRightBelt));
                }
            }, new MotorChecker.CheckerConfig() {
                {
                    mOutputPercent = 0.5;
                    mRuntime = 1;
                    mWaittime = 0.3;
                    mRPMSupplier = null;
                }
            });
    }


    @Override
    public void outputTelemetry() {
        if(debug) {
            SmartDashboard.putString("Hopper State", mCurrentState.toString());

            SmartDashboard.putNumber("Index Supply Current", mIndexMotor.getSupplyCurrent());
            SmartDashboard.putNumber("Index Stator Current", mIndexMotor.getStatorCurrent());
            SmartDashboard.putNumber("Index Output", mIndexMotor.getLastSet());

            SmartDashboard.putNumber("Left Belt Supply Current", mLeftBelt.getSupplyCurrent());
            SmartDashboard.putNumber("Left Belt Stator Current", mLeftBelt.getStatorCurrent());
            SmartDashboard.putNumber("Left Belt Output", mLeftBelt.getLastSet());
            
            SmartDashboard.putNumber("Right Belt Supply Current", mRightBelt.getSupplyCurrent());
            SmartDashboard.putNumber("Right Belt Stator Current", mRightBelt.getStatorCurrent());
            SmartDashboard.putNumber("Right Belt Output", mRightBelt.getLastSet());
        }
    }
}
