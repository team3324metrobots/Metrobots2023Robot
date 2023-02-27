// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team3324.robot.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.team3324.robot.util.Constants;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
  // --- ARM MOTORS ---
  private final CANSparkMax lMotor = new CANSparkMax(Constants.Arm.ARM_MOTOR_L, MotorType.kBrushless);
  private final CANSparkMax rMotor = new CANSparkMax(Constants.Arm.ARM_MOTOR_R, MotorType.kBrushless);
  private final CANSparkMax teleMotor = new CANSparkMax(Constants.Arm.TELESCOPE_MOTOR, MotorType.kBrushless);

  // --- ARM ENCODERS ---
  private final RelativeEncoder lEncoder = lMotor.getEncoder();
  private final RelativeEncoder rEncoder = rMotor.getEncoder();
  private final RelativeEncoder teleEncoder = teleMotor.getEncoder();

  // --- ARM FEEDFORWARD CONTROLLER ---
  private double kS;
  private double kG;
  private double kV;
  private ArmFeedforward FeedforwardArm = new ArmFeedforward(kS, kG, kV);

  /** Creates a new Arm. */
  public Arm() {
    rMotor.follow(lMotor, true);

    lMotor.setIdleMode(IdleMode.kBrake);
    rMotor.setIdleMode(IdleMode.kBrake);

    lMotor.burnFlash();
    rMotor.burnFlash();
  }

  // --- GETTERS & SETTERS ---

  public void setArmSpeed(double speed) {
    lMotor.set(speed);
  }

  public void setTeleSpeed(double speed) {
    teleMotor.set(speed);
  }

  public double getArmPosition() {
    return (lEncoder.getPosition() - rEncoder.getPosition()) / 2;
  }

  public double getTelePosition() {
    return teleEncoder.getPosition();
  }

  public double getArmVelocity() {
    return (lEncoder.getVelocity() - rEncoder.getVelocity()) / 2;
  }

  public double getFeedForwardSpeed() {
    return FeedforwardArm.calculate(getArmPosition(), getArmVelocity());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
