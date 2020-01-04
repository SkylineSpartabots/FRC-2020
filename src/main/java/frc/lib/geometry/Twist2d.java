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
public class Twist2d {
    protected static final Twist2d kIdentity = new Twist2d(0.0, 0.0, 0.0);

    public static Twist2d identity() {
        return kIdentity;
    }

    public final double dx;
    public final double dy;
    public final double dtheta; // Radians!

    public Twist2d(double dx, double dy, double dtheta) {
        this.dx = dx;
        this.dy = dy;
        this.dtheta = dtheta;
    }

    public Twist2d scaled(double scale) {
        return new Twist2d(dx * scale, dy * scale, dtheta * scale);
    }

    public double norm() {
        // Common case of dy == 0
        if (dy == 0.0)
            return Math.abs(dx);
        return Math.hypot(dx, dy);
    }

    public double curvature() {
        if (Math.abs(dtheta) < Util.kEpsilon && norm() < Util.kEpsilon)
            return 0.0;
        return dtheta / norm();
    }

    @Override
    public String toString() {
        final DecimalFormat fmt = new DecimalFormat("#0.000");
        return "(" + fmt.format(dx) + "," + fmt.format(dy) + "," + fmt.format(Math.toDegrees(dtheta)) + " deg)";
    }
}
