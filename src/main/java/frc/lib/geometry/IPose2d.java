/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.lib.geometry;

/**
 * Add your docs here.
 */
public interface IPose2d<T> extends IRotation2d<T>, ITranslation2d<T> {
    Pose2d getPose();

    T transformBy(Pose2d transform);

    T mirror();
}
