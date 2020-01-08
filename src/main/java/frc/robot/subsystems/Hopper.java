/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

/**
 * Add your docs here.
 */
public class Hopper extends Subsystem {

    private static Hopper mInstance = null;
    
    public static Hopper getInstance() {
        if(mInstance == null) {
            mInstance = new Hopper();
        }
        return mInstance;
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
