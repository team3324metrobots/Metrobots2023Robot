// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.util.Constants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
  // --- ARM MOTORS ---
  private final CANSparkMax lMotor = new CANSparkMax(Constants.Arm.ARM_MOTOR_L, MotorType.kBrushless);
  private final CANSparkMax rMotor = new CANSparkMax(Constants.Arm.ARM_MOTOR_R, MotorType.kBrushless);
  private final CANSparkMax teleMotor = new CANSparkMax(Constants.Arm.TELESCOPE_MOTOR, MotorType.kBrushless);

  // --- ARM ENCODERS ---
  private final RelativeEncoder lEncoder = lMotor.getEncoder();
  private final RelativeEncoder rEncoder = rMotor.getEncoder();

  // --- ARM PID CONTROLLER ---
  double kP = 0.0;
  double kI = 0.0;
  double kD = 0.0;
  public PIDController PIDControlArm = new PIDController(kP, kI, kD);

  /** Creates a new Arm. */
  public Arm() {
    rMotor.follow(lMotor, true);
  }

  // --- GETTERS & SETTERS ---

  public void setArmSpeed(double speed) {
    lMotor.set(speed);
  }

  public double getPosition() {
    return (lEncoder.getPosition() - rEncoder.getPosition()) / 2;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
