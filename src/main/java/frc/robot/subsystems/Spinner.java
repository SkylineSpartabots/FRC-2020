/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import frc.lib.sensors.ColorSensor;

/**
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

    //hardware
    

    private Spinner() {
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
