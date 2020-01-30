
package frc.lib.util;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


/**
 * Add your docs here.
 */
public class PIDController {
    private double kP, kI, kD;
    private double desiredValue;
    protected double prevError;
    private double errorSum;
    private double minOutput, maxOutput;
    protected boolean debug;
    private double lastTime;
    private double deltaTime;
    private double iRange;
    private DoubleSupplier sensorInput;
    private Timer finishedTimer;
    private double finishedTime;

    public PIDController(double kP, double kI, double kD, DoubleSupplier sensorInput, double finishedTime) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.finishedTime = finishedTime;
        this.sensorInput = sensorInput;
        desiredValue = 0.0;
        maxOutput = 1.0;
        debug = false;
        minOutput = -1;
        iRange = Double.MAX_VALUE; //this is a default value to always apply I until explicity told not to
    }

    public void setConstants(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }


    public void setDesiredValue(double desiredValue) {
        this.desiredValue = desiredValue;
    }

    public void enableDebug() {
        debug = true;
    }

    public void disableDebug() {
        debug = false;
    }

    public void setMinMaxOutput(double minOutput, double maxOutput) {
        this.minOutput = minOutput;
        this.maxOutput = maxOutput;
    }


    public void resetErrorSum() {
        errorSum = 0.0;
    }

    public double getDesiredValue() {
        return desiredValue;
    }

    public void setIRange(double iRange) {
        this.iRange = iRange;
    }

    public double getIRange() {
        return this.iRange;
    }

    public void reset() {
        resetErrorSum();
        prevError = 0;
        lastTime = Timer.getFPGATimestamp();
        deltaTime = 20.0;
    }


    public double getError() { 
        return desiredValue - sensorInput.getAsDouble();
    }

    public double getOutput() {
        double pVal = 0;
        double iVal = 0;
        double dVal = 0;

        double error = desiredValue - sensorInput.getAsDouble();

        
        double currentTime = Timer.getFPGATimestamp();
        deltaTime = currentTime - lastTime;
        lastTime = currentTime;
        

       // deltaTime /= 20;

        //Calculate p value
        pVal = error * kP;

        //Calculate i value
        if(Math.abs(error) < Math.abs(iRange)) {
            this.errorSum += error*deltaTime;
        } else {
            errorSum = 0;
        }
        iVal = errorSum * kI;

        //Calculate d value
        double derivative = (error - prevError) / deltaTime;
        dVal = kD * derivative;

        //overall pid output
        double output  = pVal +iVal + dVal;

        //limit value
        output = Util.limit(output, minOutput, maxOutput);

        prevError = error;

        if(debug) {
            SmartDashboard.putNumber("kp", kP);
            SmartDashboard.putNumber("kI", kI);
            SmartDashboard.putNumber("kD", kD);
            SmartDashboard.putNumber("P Out", pVal);
            SmartDashboard.putNumber("I Out", iVal);
            SmartDashboard.putNumber("D Out", dVal);
            SmartDashboard.putNumber("PID Output", output);
            SmartDashboard.putNumber("Error", error);
            SmartDashboard.putNumber("Error Sum", errorSum);
        }

        return output;
    }


    
}
