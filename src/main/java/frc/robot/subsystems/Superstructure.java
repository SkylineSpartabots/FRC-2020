/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import frc.robot.states.TimedLEDState;

/**
 * Add your docs here.
 */
public class Superstructure extends Subsystem {

    private static Superstructure mInstance = null;

    public static Superstructure getInstance() {
        if(mInstance == null) {
            mInstance = new Superstructure();
        }
        return mInstance;
    }

    private LED led;

    private Superstructure() {
        led = LED.getInstance();
        testLED();
    }

    private void testLED() {
        led.addStateToQueue(TimedLEDState.StaticLEDState.kEnabled);
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
