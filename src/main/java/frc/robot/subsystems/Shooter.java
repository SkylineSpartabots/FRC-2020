/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;

import edu.wpi.cscore.CameraServerJNI.TelemetryKind;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.drivers.LazyTalonFX;
import frc.lib.drivers.PheonixUtil;
import frc.lib.drivers.TalonFXFactory;
import frc.lib.drivers.TalonFXUtil;
import frc.lib.util.TelemetryUtil;
import frc.lib.util.TelemetryUtil.PrintStyle;
import frc.robot.Constants;
import frc.robot.Ports;
import frc.robot.loops.ILooper;
import frc.robot.loops.Loop;

/**
 * Add your docs here.
 */
public class Shooter extends Subsystem {

    private Shooter mInstance = null;

    public Shooter getInstance() {
        if (mInstance == null) {
            mInstance = new Shooter();
        }
        return mInstance;
    }

    private LazyTalonFX shootMotor;

    private PeriodicIO mPeriodicIO = new PeriodicIO();

    private State mShooterState = State.OFF;

    private synchronized void configMotorForShoot(LazyTalonFX motor, InvertType inverted, boolean sensorPhase) {
        motor.setInverted(inverted);
        PheonixUtil.checkError(motor.configSelectedFeedbackCoefficient(Constants.kShooterSensorCoefficient, 0, Constants.kTimeOutMs), 
        "failed to set shooter sensor coefficient", true); 
        motor.setSensorPhase(sensorPhase);
    }

    private Shooter() {
        shootMotor = TalonFXFactory.createDefaultFalcon("Shooter Motor", Ports.SHOOT_MOTOR_ID);
        configMotorForShoot(shootMotor, InvertType.None, true);

    }

    @Override
    public void readPeriodicInputs() {
        mPeriodicIO.encoder_counts = shootMotor.getSelectedSensorPosition(0);
    }

    @Override
    public void writePeriodicOutputs() {
        if (mShooterState == State.VELOCITY_CONTROLLED) {
            shootMotor.set(ControlMode.Velocity, mPeriodicIO.motor_demand);
        }
    }

    private enum State { //
        OFF, VELOCITY_CONTROLLED;
    }

    public static class PeriodicIO {
        //inputs
        public double encoder_counts;


        //outputs
        public double motor_demand;
    }

    private void setState(State newState) {
        mShooterState = newState;
    }

    @Override
    public void stop() {
        setState(State.OFF);
    }

    @Override
    public boolean checkSystem() {
        // TODO Auto-generated method stub
        return false;
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putNumber("Shooter encoder counts", mPeriodicIO.encoder_counts);

    }

    @Override
    public void registerEnabledLoops(ILooper loop) {
        loop.register(new Loop(){
            
            @Override
            public void onStart(double timestamp) {
                synchronized (Shooter.this) {
                    stop();
                    handleFaults();
                }                
            }

            @Override
            public void onLoop(double timestamp) {
                synchronized (Shooter.this) {
                    handleFaults();
                    switch (mShooterState) {
                        case OFF:
                            break;
                        case VELOCITY_CONTROLLED:

                            break;
                        default:
                            TelemetryUtil.print("Shooter in an unexpected control state", PrintStyle.ERROR, false);
                    }
                }                
            }

            @Override
            public void onStop(double timestamp) {
                stop();   
            }
        
        });
    }

    @Override
    public void zeroSensors() {
        PheonixUtil.checkError(shootMotor.setSelectedSensorPosition(0, 0, Constants.kTimeOutMs),"Shooter encoder not reset properly", true); 
    }

    public void handleFaults() {
        //bruh these faults are kinda crazy tho
        TalonFXUtil.checkSensorFaults(shootMotor);
        TalonFXUtil.checkMotorFaults(shootMotor);
    }
}
