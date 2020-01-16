/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.lib.drivers;

import com.revrobotics.CANError;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;

/**
 * Add your docs here.
 */

public class LazySparkMax extends CANSparkMax {

    protected double mLastSet = Double.NaN;
    protected ControlType mLastMode = null;
    protected String mName = "";

    protected CANSparkMax mLeader = null;

    public LazySparkMax(String name, int deviceID) {
        super(deviceID, MotorType.kBrushless);
        mName = name;
    }

    public CANSparkMax getLeader() {
        return mLeader;
    }

    public String getName() {
        return mName;
    }

    @Override
    public CANError follow(final CANSparkMax leader) {
        mLeader = leader;
        return super.follow(leader);
    }

    public void set(ControlType type, double setpoint) {
        if(setpoint != mLastSet || type != mLastMode) {
            mLastSet = setpoint;
            mLastMode = type;
            super.getPIDController().setReference(setpoint, type);
        }
    }

    public void set(ControlType type, double setpoint, int pidSlot) {
        if(setpoint != mLastSet || type != mLastMode) {
            mLastSet = setpoint;
            mLastMode = type;
            super.getPIDController().setReference(setpoint, type, pidSlot);
        }
    }
}
