/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import frc.lib.drivers.LazyTalonSRX;
import frc.lib.drivers.PheonixUtil;
import frc.lib.drivers.TalonSRXFactory;
import frc.lib.drivers.TalonSRXUtil;
import frc.robot.Constants;
import frc.robot.Ports;

/**
 * Add your docs here.
 */
public class Intake extends Subsystem {

    private static Intake mInstance = null;

    public static Intake getInstance() {
        if(mInstance == null) {
            mInstance = new Intake();
        }
        return mInstance;
    }

    //hardware
    private final Solenoid mIntakeSolenoid;
    private final LazyTalonSRX mIntakeMotor;

    private void configureIntakeMotor(LazyTalonSRX talon) {
        PheonixUtil.checkError(talon.configVoltageCompSaturation(12.0, Constants.kTimeOutMs),
            talon.getName() + " failed to set voltage compensation", true);
        PheonixUtil.checkError(talon.configVoltageMeasurementFilter(32, Constants.kTimeOutMs),
            talon.getName() + " failed to set voltage meas. filter", true);
        talon.enableVoltageCompensation(true);

        TalonSRXUtil.setCurrentLimit(talon, 20);
    }

    private Intake() {
        mIntakeSolenoid = new Solenoid(Ports.INTAKE_SOLENOID_PORT);
        mIntakeMotor = TalonSRXFactory.createDefaultTalon("Intake Motor", Ports.INTAKE_MOTOR_ID);
        configureIntakeMotor(mIntakeMotor);
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
