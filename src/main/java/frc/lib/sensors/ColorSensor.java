/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.lib.sensors;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;

/**
 * Add your docs here. 
 */
public class ColorSensor {

    private final Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
    private final Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
    private final Color kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.144);
    private final Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);

    private final ColorSensorV3 mColorSensor;
    private final ColorMatch mColorMatcher;

    public enum Colors {
        BLUE, GREEN, RED, YELLOW, UNKNOWN;
    }

    public ColorSensor() {
        mColorSensor = new ColorSensorV3(I2C.Port.kOnboard);
        
        mColorMatcher = new ColorMatch();
        mColorMatcher.addColorMatch(kBlueTarget);
        mColorMatcher.addColorMatch(kGreenTarget);
        mColorMatcher.addColorMatch(kRedTarget);
        mColorMatcher.addColorMatch(kYellowTarget);

        
    }

    public Colors getColor() {
        Color detectedColor = mColorSensor.getColor();
        ColorMatchResult match = mColorMatcher.matchClosestColor(detectedColor);

        if(match.color == kBlueTarget) {
            return Colors.BLUE;
        } else if (match.color == kRedTarget) {
            return Colors.RED;
        } else if (match.color == kGreenTarget) {
            return Colors.GREEN;
        } else if (match.color == kYellowTarget) {
            return Colors.YELLOW;
        } else {
            return Colors.UNKNOWN;
        }
    }



}
