package frc.team6300.NorthwoodDrivers;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController.AccelStrategy;

import edu.wpi.first.math.util.Units;


public class LoggedNeo implements LoggedMotor{
    private final CANSparkMax motor; 
    private final SparkMaxPIDController controller;
    private final RelativeEncoder encoder; 
    private final double gearRatio;

    public LoggedNeo(int motorID, boolean motorInvert, int currentLimit){
      gearRatio = 1.0;
      motor = new CANSparkMax(motorID, MotorType.kBrushless);
      controller = this.motor.getPIDController();
      encoder = motor.getEncoder();
      motor.setInverted(motorInvert);
      motor.enableVoltageCompensation(12.0);
      motor.setSmartCurrentLimit(currentLimit);
      controller.setFeedbackDevice(encoder);
      encoder.setPositionConversionFactor(gearRatio);
      encoder.setMeasurementPeriod(8);

  } 
    @Override
    public void updateInputs(LoggedMotorIOInputs inputs) {
    inputs.positionRad = Units.rotationsToRadians(
        encoder.getPosition());
    inputs.velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(
        encoder.getVelocity());
    inputs.appliedVolts = motor.getBusVoltage()*motor.getAppliedOutput();
    inputs.statorAmps = new double[] { motor.getOutputCurrent()};
    inputs.statorTempCelcius = new double[]{motor.getMotorTemperature()};
    }
    @Override
    public void setVelocity(double velocityRadPerSec, double ffVolts, int slotID){
        double velocity = Units.radiansPerSecondToRotationsPerMinute(velocityRadPerSec);
        controller.setReference(velocity, ControlType.kVelocity, slotID);
    }
    @Override
    public void setPosition(double positionRad, int slotID){
        controller.setReference(positionRad, ControlType.kPosition, slotID);
    }
    @Override
    public void stop() {
      motor.set(0.0);
    }
    public void setSmartMotionPosition(double positionRad, int slotID){
      controller.setReference(positionRad, ControlType.kSmartMotion, slotID);
    }
    public void setSmartVelocity(double velocityRadPerSec, int slotID){
      controller.setReference(velocityRadPerSec, ControlType.kSmartVelocity, slotID);
    }
    public void setPercentOutput(double percent){
      motor.set(percent);
    }
    @Override
    public void configurePID(double kP, double kI, double kD, double ff, int slotID) {
    controller.setP(kP, slotID);
    controller.setI(kI, slotID);
    controller.setD(kD, slotID);
    controller.setFF(ff, slotID);
    }
    public void configureSmartMotion(double maxVelocity, double maxAcceleration, double allowableError, int slotID, AccelStrategy strategy){
      controller.setSmartMotionAccelStrategy(strategy, slotID);
      controller.setSmartMotionAllowedClosedLoopError(allowableError, slotID);
      controller.setSmartMotionMaxAccel(maxAcceleration, slotID);
      controller.setSmartMotionMaxVelocity(maxVelocity, slotID);
    
    }

    public double getCurrentAmps(){
      return motor.getOutputCurrent();
    }
    @Override
    public double getPosition(){
      return Units.rotationsToRadians(encoder.getPosition());
    }
    @Override
    public double getVelocity(){
      return Units.rotationsPerMinuteToRadiansPerSecond(encoder.getVelocity());
    }
    
    


}
