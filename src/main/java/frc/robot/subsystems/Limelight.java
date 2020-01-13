/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.geometry.Rotation2d;
import frc.lib.geometry.Translation2d;
import frc.lib.util.Util;
import frc.lib.vision.TargetInfo;
import frc.robot.Constants;

/**
 * Add your docs here.
 */
public class Limelight extends Subsystem {
    
    public static class LimelightConstants {
        public String kName = "";
        public String kTableName = "";
        public double kHeight = 0.0;
        public Rotation2d kHorizontalToLens = Rotation2d.identity();
    }

    private NetworkTable mNetworkTable;
    private LimelightConstants mConstants = null;

    public Limelight(LimelightConstants constants) {
        mConstants = constants;
        mNetworkTable = NetworkTableInstance.getDefault().getTable(mConstants.kTableName);
    }

    public static class PeriodicIO {
        //Inputs
        public double latency;
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
    private double[] mZeroArray = new double[]{0, 0, 0, 0, 0, 0, 0, 0};
    private List<TargetInfo> mTargets = new ArrayList<>();
    private boolean mSeesTarget = false;

    public double getLensHeight() {
        return mConstants.kHeight;
    }

    public Rotation2d getHorizontalToLens() {
        return mConstants.kHorizontalToLens;
    }

    @Override
    public synchronized void readPeriodicInputs() {
        mPeriodicIO.latency = mNetworkTable.getEntry("tl").getDouble(0) / 1000.0 + Constants.kImageCaptureLatency;
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
        SmartDashboard.putBoolean(mConstants.kName + ": Has Target", mSeesTarget);
        SmartDashboard.putNumber(mConstants.kName + ": X-Offset", mPeriodicIO.xOffset);
        SmartDashboard.putNumber(mConstants.kName + ": Y-Offset", mPeriodicIO.yOffset);
        SmartDashboard.putNumber(mConstants.kName + ": Area", mPeriodicIO.area);
    }

    public enum LedMode {
        PIPELINE, OFF, BLINK, ON;
    }

    public synchronized double getYOffset() {
        return mPeriodicIO.yOffset;
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
        return (Constants.kTargetHeight - mConstants.kHeight) / 
            Math.tan(Math.toRadians(mConstants.kHorizontalToLens.getDegrees() - mPeriodicIO.yOffset));
    }

    public synchronized List<TargetInfo> getTarget() {
        List<TargetInfo> targets = getRawTargetInfos();
        if(seesTarget() && targets != null) {
            return targets;
        }

        return null;
    }

    public synchronized List<TargetInfo> getRawTargetInfos() {
        List<double[]> corners = getTopCorners();
        if(corners == null) {
            return null;
        }

        double slope = 1.0;
        if (Math.abs(corners.get(1)[0] - corners.get(0)[0]) > Util.kEpsilon) {
            slope = (corners.get(1)[1] - corners.get(0)[1]) /
                    (corners.get(1)[0] - corners.get(0)[0]);
        }

        mTargets.clear();
        for (int i = 0; i < 2; ++i) {
            // Average of y and z;
            double y_pixels = corners.get(i)[0];
            double z_pixels = corners.get(i)[1];

            // Redefine to robot frame of reference.
            double nY = -((y_pixels - 160.0) / 160.0);
            double nZ = -((z_pixels - 120.0) / 120.0);

            double y = Constants.kVPW / 2 * nY;
            double z = Constants.kVPH / 2 * nZ;

            TargetInfo target = new TargetInfo(y, z);
            target.setSkew(slope);
            mTargets.add(target);
        }

        return mTargets;
    }

    private List<double[]> getTopCorners() {
        double[] xCorners = mNetworkTable.getEntry("tcornx").getDoubleArray(mZeroArray);
        double[] yCorners = mNetworkTable.getEntry("tcorny").getDoubleArray(mZeroArray);
        mSeesTarget = mNetworkTable.getEntry("tv").getDouble(0.0) == 1.0;
        
        if(!mSeesTarget || Arrays.equals(xCorners, mZeroArray) || Arrays.equals(yCorners, mZeroArray)
            || xCorners.length != 4 || yCorners.length != 4) {
                return null;
        }

        return extractCornersFromBoundingBoxes(xCorners, yCorners);
    }

    private static final Comparator<Translation2d> xSort = Comparator.comparingDouble(Translation2d::x);
    private static final Comparator<Translation2d> ySort = Comparator.comparingDouble(Translation2d::y);

    public static List<double[]> extractCornersFromBoundingBoxes(double[] xCorners, double[] yCorners) {
        List<Translation2d> corners = new ArrayList<>();
        for(int i = 0; i < xCorners.length; i++) {
            corners.add(new Translation2d(xCorners[i], yCorners[i]));
        }

        corners.sort(xSort);

        List<Translation2d> leftSide = corners.subList(0, 2);
        List<Translation2d> rightSide = corners.subList(2, 4);

        leftSide.sort(ySort);
        rightSide.sort(ySort);

        Translation2d leftCorner = leftSide.get(0);
        Translation2d rightCorner = rightSide.get(0);

        return List.of(new double[]{leftCorner.x(), leftCorner.y()}, 
            new double[]{rightCorner.x(), rightCorner.y()});
    }


    @Override
    public void stop() {

    }

    @Override
    public boolean checkSystem() {
        return false;
    }

}
