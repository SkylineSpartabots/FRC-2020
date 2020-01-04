/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.lib.geometry;

import java.text.DecimalFormat;

import frc.lib.util.Util;

/**
 * Add your docs here.
 */
public class Displacement1d implements State<Displacement1d>{

    protected final double displacement_;

    public Displacement1d() {
        displacement_ = 0.0;
    }

    public Displacement1d(double displacement) {
        displacement_ = displacement;
    }

    public double x() {
        return displacement_;
    }

    @Override
    public Displacement1d interpolate(Displacement1d other, double x) {
        return new Displacement1d(Util.interpolate(displacement_, other.displacement_, x));
    }

    @Override
    public double distance(Displacement1d other) {
        return Math.abs(x() - other.x());
    }

    @Override
    public String toString() {
        final DecimalFormat fmt = new DecimalFormat("#0.000");
        return fmt.format("(" + x() + ")");
    }

    @Override
    public String toCSV() {
        final DecimalFormat fmt = new DecimalFormat("#0.000");
        return fmt.format(x());
    }
}
