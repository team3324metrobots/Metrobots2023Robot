package frc.team3324.library.motorcontrollers;

import com.revrobotics.CANSparkMax; 
import com.revrobotics.RelativeEncoder;

public class SmartMotionSparkMAX extends CANSparkMax {
    public SmartMotionSparkMAX(int id, MotorType type, int smartCurrentLimit, double rampRate, double kP, double kI, double kD, double kF) {
        super(id, type);
        setCurrentLimit(smartCurrentLimit);

        super.getPIDController().setP(kP);
        super.getPIDController().setI(kI);
        super.getPIDController().setD(kD);
        super.getPIDController().setFF(kF);

        super.setOpenLoopRampRate(rampRate);
        super.setClosedLoopRampRate(rampRate);
    }

    public void follow(SmartMotionSparkMAX leader, boolean invert) {
        super.follow(leader, invert);
    }

    public void setCurrentLimit(int value) {
        super.setSmartCurrentLimit(value);
    }

    public double getCurrentDraw() {
        return this.getOutputCurrent();
    }

    @Override
    public RelativeEncoder getEncoder() {
        return super.getEncoder();
    }
}
