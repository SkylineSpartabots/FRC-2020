/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.drivers.LazySparkMax;
import frc.lib.drivers.SparkMaxFactory;
import frc.lib.drivers.SparkMaxUtil;
import frc.lib.sensors.ColorSensor;
import frc.lib.sensors.ColorSensor.Colors;
import frc.lib.util.PIDController;
import frc.lib.util.TelemetryUtil;
import frc.lib.util.TelemetryUtil.PrintStyle;
import frc.robot.Constants;
import frc.robot.Ports;
import frc.robot.loops.ILooper;
import frc.robot.loops.Loop;
import frc.robot.subsystems.requests.Request;

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
    private final PIDController mPidController;
    private double mEncoderTarget;
    private Colors mColorTarget = Colors.UNKNOWN;

    //control states
    private SpinnerControlState mCurrentState;
    private boolean mStateChanged = false;
    private double mStateChangeTimestamp = 0;


    private void configureSparkForSpinner(LazySparkMax spark) {
        SparkMaxUtil.checkError(spark.enableVoltageCompensation(12.0), spark.getName() + " failed to set voltage comp.", true);
        SparkMaxUtil.checkError(spark.setSmartCurrentLimit(20), spark.getName() + " failed to set current limit", true);
        SparkMaxUtil.checkError(spark.setOpenLoopRampRate(0.5), spark.getName() + " failed to set open loop ramp", true);
        SparkMaxUtil.checkError(spark.setClosedLoopRampRate(0.4), spark.getName() + " failed to set closed loop ramp", true);
    }


    private Spinner() {
        mSpinnerMotor = SparkMaxFactory.createDefaultSparkMax("Spinner Motor", Ports.SPINNER_MOTOR_ID, false);
        configureSparkForSpinner(mSpinnerMotor);

        mEncoder = mSpinnerMotor.getEncoder();

        mPidController = new PIDController(0.0, 0.0, 0.0);
        mPidController.setTolerance(25);
        mPidController.setMinMaxOutput(-1.0, 1.0);

        mSpinnerSolenoid = new Solenoid(Ports.PCM_ID, Ports.SPINNER_SOLENOID_ID);

        mColorSensor = ColorSensor.getInstance();
    }


    @Override
    public void registerEnabledLoops(ILooper mEnabledLooper) {
        mEnabledLooper.register(new Loop() {

            @Override
            public void onStart(double timestamp) {
                setOff();
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

    private void setState(SpinnerControlState newState) {
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

    //rotation control uses bang-bang for speed, overshoot does not matter due to 3-5 rotation window
    public synchronized void setRotationControl() {
        if(mCurrentState != SpinnerControlState.ROTATION_CONTROL) {
            mSpinnerSolenoid.set(true);
            mEncoderTarget = mEncoder.getPosition() + (Constants.kCountsPerControlPanelRotation * 3.1);
            setState(SpinnerControlState.ROTATION_CONTROL);
        }
        mSpinnerMotor.set(ControlType.kDutyCycle, Constants.kRotationControlPercentOutput);
    }

    private void updateRotationControl() {
        if(mCurrentState == SpinnerControlState.ROTATION_CONTROL) {
            if(mEncoder.getPosition() > mEncoderTarget) {
                setOff();
            }
        } else {
            TelemetryUtil.print("Spinner is not in the rotation control state", PrintStyle.ERROR, true);
        }
    }

    public synchronized void setPositionControl() {
        if(mCurrentState != SpinnerControlState.POSITION_CONTROL) {
            getPositionControlColor();
            if(mColorTarget == Colors.UNKNOWN) {
                setOff();
                return;
            }
            mSpinnerSolenoid.set(true);
            setState(SpinnerControlState.POSITION_CONTROL);
        }
    }

    private void updatePositionControl() {
        if(mCurrentState == SpinnerControlState.POSITION_CONTROL) {
           
            

        } else {
            TelemetryUtil.print("Spinner is not in the position control state", PrintStyle.ERROR, true);
        }
    }

    public synchronized void getPositionControlColor() {
        String gameData = DriverStation.getInstance().getGameSpecificMessage();
        if(gameData.length() > 0) {
            switch(gameData.charAt(0)) {
                case 'B':
                    mColorTarget = Colors.BLUE;
                    break;
                case 'G':
                    mColorTarget = Colors.GREEN;
                    break;
                case 'R':
                    mColorTarget = Colors.RED;
                    break;
                case 'Y':
                    mColorTarget = Colors.YELLOW;
                    break;
                default:
                    mColorTarget = Colors.UNKNOWN;
                    TelemetryUtil.print("Corrupt game data recieved", PrintStyle.ERROR, true);
            }
        } else {
            mColorTarget = Colors.UNKNOWN;
        }

        SmartDashboard.putString("Spinner Color:", mColorTarget.toString());
        
    }


    public Request rotationControlRequest() {
        return new Request(){
        
            @Override
            public void act() {
                setRotationControl();
            }
        };
    }

    public Request positionControlRequest() {
        return new Request(){
        
            @Override
            public void act() {
                setPositionControl();
            }
        };
    }

    @Override
    public void stop() {
        setOff();
    }

    @Override
    public boolean checkSystem() {
        return false;
    }

    @Override
    public void outputTelemetry() {
        if(debug) {

        }
    }
}
