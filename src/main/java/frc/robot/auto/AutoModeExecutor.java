/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.auto;

import frc.lib.util.CrashTrackingRunnable;
import frc.robot.auto.modes.AutoModeBase;

/**
 * Add your docs here.
 */
public class AutoModeExecutor {
    private static AutoModeExecutor mInstance = null;

    private AutoModeBase mAutoMode = null;
    private Thread mThread = null;

    public AutoModeExecutor() {}

    public static AutoModeExecutor getInstance() {
        if(mInstance == null) {
            mInstance = new AutoModeExecutor();
        }

        return mInstance;
    }

    public void setAutoMode(AutoModeBase new_auto_mode) {
        mAutoMode = new_auto_mode;
        mThread = new Thread(new CrashTrackingRunnable(){
        
            @Override
            public void runCrashTracked() {
                if(mAutoMode != null) {
                    mAutoMode.run();
                }
            }
        });
    }

    public void start() {
        if (mThread != null) {
            mThread.start();
        }
    }

    public boolean isStarted() {
        return mAutoMode != null && mAutoMode.isActive() && mThread != null && mThread.isAlive();
    }

    public void reset() {
        if (isStarted()) {
            stop();
        }

        mAutoMode = null;
    }

    public void stop() {
        if (mAutoMode != null) {
            mAutoMode.stop();
        }

        mThread = null;
    }

    public AutoModeBase getAutoMode() {
        return mAutoMode;
    }

    public boolean isInterrupted() {
        if (mAutoMode == null) {
            return false;
        }
        return mAutoMode.getIsInterrupted();
    }

    public void interrupt() {
        if (mAutoMode == null) {
            return;
        }
        mAutoMode.interrupt();
    }

    public void resume() {
        if (mAutoMode == null) {
            return;
        }
        mAutoMode.resume();
    }
}
