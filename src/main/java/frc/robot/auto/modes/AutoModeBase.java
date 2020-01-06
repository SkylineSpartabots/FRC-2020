/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.auto.modes;

import frc.lib.util.TelemetryUtil;
import frc.lib.util.TelemetryUtil.PrintStyle;
import frc.robot.auto.AutoModeEndedException;
import frc.robot.auto.actions.Action;

/**
 * Add your docs here.
 */
public abstract class AutoModeBase {
    protected final double mUpdateRate = 1.0/50.0;
    protected boolean mActive = false;
    protected boolean mIsInterrupted = false;

    protected abstract void routine() throws AutoModeEndedException;

    public void run() {
        mActive = true;

        try {
            routine();
        } catch(AutoModeEndedException e) {
            TelemetryUtil.print("Auto mode ended early", PrintStyle.ERROR, false);
            return;
        }

        done();
    }

    public void done() {
        TelemetryUtil.print("Auto mode finished", PrintStyle.INFO, false);
    }

    public void stop() {
        mActive = false;
    }

    public void interrupt() {
        TelemetryUtil.print("Auto mode interrupted", PrintStyle.INFO, false);
        mIsInterrupted = true;
    }

    public void resume() {
        TelemetryUtil.print("Auto mode resumed", PrintStyle.INFO, false);
        mIsInterrupted = false;
    }

    public boolean isActive() {
        return mActive;
    }

    public boolean isActiveWithThrow() throws AutoModeEndedException {
        if(!isActive()) {
            throw new AutoModeEndedException();
        }

        return isActive();
    }

    public void runAction(Action action) throws AutoModeEndedException {
        isActiveWithThrow();
        long waitTime = (long) (mUpdateRate * 1000.0);

        while (isActiveWithThrow() && mIsInterrupted) {
            try {
                Thread.sleep(waitTime);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }

        action.start();

        while(isActiveWithThrow() && !action.isFinished() && !mIsInterrupted) {
            action.update();

            try {
                Thread.sleep(waitTime);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }

        action.done();
    }

    public boolean getIsInterrupted() {
        return mIsInterrupted;
    }



}