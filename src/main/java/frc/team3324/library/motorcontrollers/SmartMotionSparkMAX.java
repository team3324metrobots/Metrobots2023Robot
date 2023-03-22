package frc.team3324.library.motorcontrollers;

import com.revrobotics.CANSparkMax; 
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.SparkMaxPIDController.AccelStrategy;

import frc.team6300.NorthwoodDrivers.LoggedNeo;

public class SmartMotionSparkMAX extends LoggedNeo {
    public SmartMotionSparkMAX(int id, boolean invert, int smartCurrentLimit, double gearRatio, double rampRate, double kP, double kI, double kD, double kF) {
        super(id, invert, smartCurrentLimit, gearRatio);
        setCurrentLimit(smartCurrentLimit);

        super.configurePID(kP, kI, kD, kF, 0);
        super.configureRampRate(rampRate, rampRate);

        super.configureSmartMotion(4, 4, 0.1, 0, AccelStrategy.kTrapezoidal);
    }

    public void follow(CANSparkMax leader) {
        super.setSlave(leader);
    }

    public void setCurrentLimit(int value) {
        super.motor.setSmartCurrentLimit(value);
    }

    public double getCurrentDraw() {
        return this.getCurrentAmps();
    }

    public CANSparkMax getMotor() {
        return super.getMotorObject();
    }

    public RelativeEncoder getEncoder() {
        return super.encoder;
    }

    public void setSmartMotionPosition(double reference) {
        super.motor.getPIDController().setReference(reference, ControlType.kSmartMotion);
    }
}
