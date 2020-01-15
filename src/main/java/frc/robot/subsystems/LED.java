/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.ArrayList;

import com.ctre.phoenix.CANifier;

import frc.robot.Ports;
import frc.robot.states.LEDState;
import frc.robot.states.TimedLEDState;

/**
 * Add your docs here.
 */
public class LED extends Subsystem {

    private static LED mInstance = null;

    public static LED getInstance() {
        if (mInstance == null) {
            mInstance = new LED();
        }
        return mInstance;
    }

    // hardware
    private final CANifier mCanifier;

    // states
    private ArrayList<TimedLEDState> states = new ArrayList<>();
    private LEDState mDesiredLEDState = new LEDState(0.0, 0.0, 0.0);



    private LED() {
        mCanifier = new CANifier(Ports.CANIFIER_ID);
    }


    public void addFormostActiveState(TimedLEDState timedState) {
        states.add(0, timedState);
    }

    public void addStateToQueue(TimedLEDState timedState) {
        states.add(timedState);
    }



    





    @Override
    public void stop() {
        // TODO Auto-generated method stub

    }

    @Override
    public boolean checkSystem() {
        // TODO Auto-generated method stub
        return false;
    }

    @Override
    public void outputTelemetry() {
        // TODO Auto-generated method stub

    }
}
