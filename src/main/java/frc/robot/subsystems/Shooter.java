/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import frc.lib.drivers.LazySparkMax;
import frc.lib.drivers.LazyTalonFX;
import frc.lib.drivers.LazyTalonSRX;
import frc.lib.drivers.SparkMaxFactory;
import frc.lib.drivers.TalonFXFactory;
import frc.lib.drivers.TalonSRXFactory;
import frc.robot.Ports;

/**
 * Add your docs here.
 */
public class Shooter extends Subsystem {

    private static Shooter mInstance = null;

    public static Shooter getInstance() {
        if(mInstance == null) {
            mInstance = new Shooter();
        }
        return mInstance;
    }

    //hardware
    private final LazySparkMax mLeftShooter, mRightShooter;
    private final LazyTalonSRX mHoodMotor, mIndexer;
    private final Solenoid mRampSolenoid;


    private Shooter() {
        mLeftShooter = SparkMaxFactory.createDefaultSparkMax("Left Shooter Neo", Ports.SHOOTER_LEFT_SHOOT_ID);
        
        mRightShooter = SparkMaxFactory.createSlaveSparkMax("Right Shooter Neo", Ports.SHOOTER_RIGHT_SHOOT_ID,
             mLeftShooter);

        
        mHoodMotor = TalonSRXFactory.createDefaultTalon("Hood Motor", Ports.SHOOTER_HOOD_ID);
        mIndexer = TalonSRXFactory.createDefaultTalon("Index Motor", Ports.SHOOTER_INDEX_ID);
        
        mRampSolenoid = new Solenoid(Ports.SHOOTER_RAMP_SOLENOID_PORT);
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
