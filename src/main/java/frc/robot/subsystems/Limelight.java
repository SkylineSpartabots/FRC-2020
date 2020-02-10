/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

/**
 * Add your docs here.
 */
public class Limelight extends Subsystem {

    private static Limelight mInstance = null;
    public static Limelight getInstance() {
        if(mInstance == null) {
            mInstance = new Limelight();
        }
        return mInstance;
    }
    
    private NetworkTable mNetworkTable;

    private Limelight() {
        mNetworkTable = NetworkTableInstance.getDefault().getTable("limelight");
    }

    public static class PeriodicIO {
        //Inputs
        public double givenLedMode;
        public int givenPipeline;
        public double xOffset;
        public double yOffset;
        public double area;

        //Outputs
        public int ledMode = 1;
        public int camMode = 0;
        public int pipeline = 0;
        public int stream = 2;
        public int snapshot = 0;
    }

    private PeriodicIO mPeriodicIO = new PeriodicIO();
    private boolean mOutputsHaveChanged = true;
    private boolean mSeesTarget = false;

    @Override
    public synchronized void readPeriodicInputs() {
        mPeriodicIO.givenLedMode = (int) mNetworkTable.getEntry("ledMode").getDouble(1.0);
        mPeriodicIO.givenPipeline = (int) mNetworkTable.getEntry("pipeline").getDouble(0);
        mPeriodicIO.xOffset = mNetworkTable.getEntry("tx").getDouble(0.0);
        mPeriodicIO.yOffset = mNetworkTable.getEntry("ty").getDouble(0.0);
        mPeriodicIO.area = mNetworkTable.getEntry("ta").getDouble(0.0);
        mSeesTarget = mNetworkTable.getEntry("tv").getDouble(0) == 1.0;
    }

    @Override
    public synchronized void writePeriodicOutputs() {
        if (mPeriodicIO.givenLedMode != mPeriodicIO.ledMode ||
                mPeriodicIO.givenPipeline != mPeriodicIO.pipeline) {
            mOutputsHaveChanged = true;
        }
        if (mOutputsHaveChanged) {
            mNetworkTable.getEntry("ledMode").setNumber(mPeriodicIO.ledMode);
            mNetworkTable.getEntry("camMode").setNumber(mPeriodicIO.camMode);
            mNetworkTable.getEntry("pipeline").setNumber(mPeriodicIO.pipeline);
            mNetworkTable.getEntry("stream").setNumber(mPeriodicIO.stream);
            mNetworkTable.getEntry("snapshot").setNumber(mPeriodicIO.snapshot);

            mOutputsHaveChanged = false;
        }
    }

    @Override
    public synchronized void outputTelemetry() {
        SmartDashboard.putBoolean("Has Target: ", mSeesTarget);
        SmartDashboard.putNumber("X-Offset: ", mPeriodicIO.xOffset);
        SmartDashboard.putNumber("Y-Offset: ", mPeriodicIO.yOffset);
    }

    public enum LedMode {
        PIPELINE, OFF, BLINK, ON;
    }

    public synchronized double getYOffset() {
        return mPeriodicIO.yOffset;
    }

    public synchronized double getXOffset() {
        return mPeriodicIO.xOffset;
    }

    public synchronized double getTargetArea() {
        return mPeriodicIO.area;
    }

    public synchronized void setLed(LedMode mode) {
        if(mode.ordinal() != mPeriodicIO.ledMode) {
            mPeriodicIO.ledMode = mode.ordinal();
            mOutputsHaveChanged = true;
        }
    }

    public synchronized void setPipeline(int pipeline) {
        if(pipeline != mPeriodicIO.pipeline) {
            //TODO: RobotState.getInstance().resetVision();
            mPeriodicIO.pipeline = pipeline;
            mOutputsHaveChanged = true;
        }
    }

    public synchronized void triggerOutputs() {
        mOutputsHaveChanged = true;
    }

    public synchronized int getPipeline() {
        return mPeriodicIO.pipeline;
    }

    public synchronized boolean seesTarget() {
        return mSeesTarget;
    }

    public synchronized double getDistance() {
        return (Constants.kTargetHeight - Constants.kLensHeight) / 
            Math.tan(Math.toRadians(Constants.kLensHorizontalAngle + mPeriodicIO.yOffset));
    }

    @Override
    public void stop() {

    }

    @Override
    public boolean checkSystem() {
        return false;
    }

}
