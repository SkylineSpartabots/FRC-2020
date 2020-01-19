/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.Solenoid;
import frc.lib.drivers.LazyTalonFX;
import frc.lib.drivers.TalonFXFactory;
import frc.lib.drivers.TalonFXUtil;
import frc.robot.Ports;

/**
 * Add your docs here.
 */
public class Climb extends Subsystem {

    private static Climb mInstance = null;

    public static Climb getInstance() {
        if(mInstance == null) {
            mInstance = new Climb();
        }
        return mInstance;
     }

     //hardware
     private final Solenoid horizontalRelease, leftVerticalRelease, rightVerticalRelease;
     private final LazyTalonFX leftClimbMotor, rightClimbMotor;

     private void configureMotorForClimb(LazyTalonFX motor) {
         TalonFXUtil.setStatorCurrentLimit(motor, 30);
         TalonFXUtil.setSupplyCurrentLimit(motor, 30);//TODO: set currents properly
     }

     private Climb() {
        horizontalRelease = new Solenoid(Ports.HORIZONTAL_RELEASE_SOLENOID);
        leftVerticalRelease = new Solenoid(Ports.LEFT_VERTICAL_RELEASE_SOLENOID);
        rightVerticalRelease = new Solenoid(Ports.RIGHT_VERTICAL_RELEASE_SOLENOID);

        leftClimbMotor = TalonFXFactory.createDefaultFalcon("Left Climb Motor", Ports.LEFT_CLIMB_MOTOR);
        rightClimbMotor = TalonFXFactory.createDefaultFalcon("Right Climb Motor", Ports.RIGHT_CLIMB_MOTOR);

     }


     

     //hardware states






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
