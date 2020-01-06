/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.lib.drivers;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.StickyFaults;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;

import frc.lib.util.TelemetryUtil;
import frc.lib.util.TelemetryUtil.PrintStyle;

/**
 * Add your docs here.
 */
public class TalonFXUtil {

    public synchronized static void setSupplyCurrentLimit(final LazyTalonFX falcon, int amps) {
        SupplyCurrentLimitConfiguration supplyCurrent = new SupplyCurrentLimitConfiguration(true, amps, amps, 0);
        
        PheonixUtil.checkError(falcon.configSupplyCurrentLimit(supplyCurrent), 
            falcon.getName() + " failed to set supply current limit", true);
    }

    public synchronized static void setStatorCurrentLimit(final LazyTalonFX falcon, int amps) {
        StatorCurrentLimitConfiguration statorCurrent = new StatorCurrentLimitConfiguration(true, amps, amps, 0);

        PheonixUtil.checkError(falcon.configStatorCurrentLimit(statorCurrent),
            falcon.getName() + " failed to set stator current limit", true);
    }

    public static synchronized void checkMotorFaults(final LazyTalonFX falcon) {
        StickyFaults faults = new StickyFaults();
        PheonixUtil.checkError(falcon.getStickyFaults(faults), 
            falcon.getName() + " failed to retrieve faults", true);
        boolean hasFaults = false;

        if(faults.UnderVoltage) {
            TelemetryUtil.print(falcon.getName() + " is under-powered", 
                PrintStyle.ERROR, true);
            hasFaults = true;
        }

        if(faults.ResetDuringEn) {
            TelemetryUtil.print(falcon.getName() + " was reset while enabled",
                PrintStyle.ERROR, true);
            hasFaults = true;
        }

        if(hasFaults) {
            PheonixUtil.checkError(falcon.clearStickyFaults(), 
                falcon.getName() + " failed to clear sticky faults", true);
        }
    }

    public static synchronized void checkSlaveFaults(final LazyTalonFX slaveFalcon, int master_ID) {
        StickyFaults faults = new StickyFaults();
        PheonixUtil.checkError(slaveFalcon.getStickyFaults(faults), 
            slaveFalcon.getName() + " failed to retrieve faults", true);
        boolean hasFaults = false;

        if(faults.UnderVoltage) {
            TelemetryUtil.print(slaveFalcon.getName() + " is under-powered", PrintStyle.ERROR,
                 true);
            hasFaults = true;
        }

        if(faults.ResetDuringEn) {
            TelemetryUtil.print(slaveFalcon.getName() + " reset while enabled", PrintStyle.ERROR,
                 true);
            slaveFalcon.set(ControlMode.Follower, master_ID);
            hasFaults = true;
        }

        if(hasFaults) {
            PheonixUtil.checkError(slaveFalcon.clearStickyFaults(), 
                slaveFalcon.getName() + " failed to clear sticky faults", true);
        }

    }

    public static synchronized void checkSensorFaults(final LazyTalonFX falcon) {
        StickyFaults faults = new StickyFaults();
        PheonixUtil.checkError(falcon.getStickyFaults(faults), 
            falcon.getName() + " failed to retrieve faults", true);
        boolean hasFaults = false;

        if(faults.UnderVoltage) {
            TelemetryUtil.print(falcon.getName() + " is under-powered", 
                PrintStyle.ERROR, true);
            hasFaults = true;
        }

        if(faults.ResetDuringEn) {
            TelemetryUtil.print(falcon.getName() + " was reset while enabled", 
                PrintStyle.ERROR, true);
            hasFaults = true;
        }

        if(faults.SensorOutOfPhase) {
            TelemetryUtil.print(falcon.getName() + " sensor is out of phase", 
                PrintStyle.ERROR, true);
            hasFaults = true;
        }

        if(faults.RemoteLossOfSignal) {
            TelemetryUtil.print(falcon.getName() + " loss remote signal", 
                PrintStyle.ERROR, true);
            hasFaults = true;
        }

        if(faults.SensorOverflow) {
            TelemetryUtil.print(falcon.getName() + "'s sensor overflowed'", 
                PrintStyle.ERROR, true);
            hasFaults = true;
        }

        if(hasFaults) {
            PheonixUtil.checkError(falcon.clearStickyFaults(), 
                falcon.getName() + " failed to clear sticky faults", true);
        }
    }
}