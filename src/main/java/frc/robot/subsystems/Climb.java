/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
        if (mInstance == null) {
            mInstance = new Climb();
        }
        return mInstance;
    }

    // hardware
    private final Solenoid leftHorizontalRelease, rightHorizontalRelease, leftVerticalRelease, rightVerticalRelease;
    private final LazyTalonFX leftClimbMotor, rightClimbMotor;

    private void configureMotorForClimb(LazyTalonFX motor) {
        TalonFXUtil.setStatorCurrentLimit(motor, 30);
        TalonFXUtil.setSupplyCurrentLimit(motor, 30);// TODO: set currents properly
    }

    private Climb() {
        leftHorizontalRelease = new Solenoid(Ports.LEFT_HORIZONTAL_RELEASE_SOLENOID);
        rightHorizontalRelease = new Solenoid(Ports.RIGHT_HORIZONTAL_RELEASE_SOLENOID);

        leftVerticalRelease = new Solenoid(Ports.LEFT_VERTICAL_RELEASE_SOLENOID);
        rightVerticalRelease = new Solenoid(Ports.RIGHT_VERTICAL_RELEASE_SOLENOID);

        leftClimbMotor = TalonFXFactory.createDefaultFalcon("Left Climb Motor", Ports.LEFT_CLIMB_MOTOR);
        rightClimbMotor = TalonFXFactory.createDefaultFalcon("Right Climb Motor", Ports.RIGHT_CLIMB_MOTOR);

        rightClimbMotor.set(ControlMode.Follower, Ports.LEFT_CLIMB_MOTOR);

    }

    private void setHorizontal(boolean Up) {
        leftHorizontalRelease.set(Up);
        rightHorizontalRelease.set(Up);
    }

    private void setVertical(boolean Up) {
        leftVerticalRelease.set(Up);
        rightVerticalRelease.set(Up);
    }
    // hardware states

    public enum State {
        Down(true, true, 0), ArmUp(false, true, 0), Extended(false, false, 0), Retracting(false, false, 0.5),
        Pause(true, true, 0);
        // pause solenoid values for pause arent used (see conformToState)

        public boolean horizontalReleased;
        public boolean verticalReleased;
        public double climbPower;

        private State(boolean horizontalReleased, boolean verticalReleased, double climbPower) {
            this.horizontalReleased = horizontalReleased;
            this.verticalReleased = verticalReleased;
            this.climbPower = climbPower;
        }
    }

    private State currentState = State.Down;

    private State getState() {
        return currentState;
    }

    private synchronized void setState(State newState) {
        if (newState != currentState)
            stateChanged = true;
        currentState = newState;
    }

    private boolean stateChanged = false;

    public void conformToState(State desiredState) {

        leftClimbMotor.set(ControlMode.PercentOutput, desiredState.climbPower);
        // and the right
        if (desiredState != State.Pause) {
            setHorizontal(desiredState.horizontalReleased);
            setVertical(desiredState.verticalReleased);
        }
        setState(desiredState);
    }
    /*
     * this is last years request code
     * 
     * public Request stateRequest(State newState) { return new Request() {
     * 
     * @Override public void act() { conformToState(newState); } }; }
     */

    /*
     * @Override public void onStop(double timestamp) { conformToState(State.Pause);
     * }
     * 
     * };
     */

    @Override
    public void outputTelemetry() {
        SmartDashboard.putBoolean("Arm Down", leftHorizontalRelease.get());
        SmartDashboard.putBoolean("Arm Extended", leftVerticalRelease.get());
        SmartDashboard.putNumber("Arm Retracting Power", leftClimbMotor.getLastSet());

    }

    @Override
    public void stop() {

    }

    @Override
    public boolean checkSystem() {
        return false;
    }

}
