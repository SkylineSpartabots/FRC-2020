/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.lib.drivers;

import com.revrobotics.CANError;

import frc.lib.util.TelemetryUtil;
import frc.lib.util.TelemetryUtil.PrintStyle;

/**
 * Add your docs here.
 */
public class SparkMaxUtil {

    public static void checkError(CANError errorCode, String message, boolean log) {
        if (errorCode != CANError.kOk) {
            TelemetryUtil.print(message + " " + errorCode, PrintStyle.ERROR, log);
        }
    }
}