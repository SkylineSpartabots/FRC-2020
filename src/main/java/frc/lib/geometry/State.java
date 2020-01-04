/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.lib.geometry;

import frc.lib.util.CSVWritable;
import frc.lib.util.Interpolable;

/**
 * Add your docs here.
 */
public interface State<S> extends Interpolable<S>, CSVWritable{
    double distance(final S other);

    boolean equals(final Object other);

    String toString();
    
    String toCSV();
}
