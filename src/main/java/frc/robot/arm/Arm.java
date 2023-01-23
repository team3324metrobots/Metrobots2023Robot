// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.util.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
  // Define arm motor controllers
  private final CANSparkMax lMotor = new CANSparkMax(Constants.Arm.ARM_MOTOR_L, MotorType.kBrushless);
  private final CANSparkMax rMotor = new CANSparkMax(Constants.Arm.ARM_MOTOR_R, MotorType.kBrushless);
  private final CANSparkMax teleMotor = new CANSparkMax(Constants.Arm.TELESCOPE_MOTOR, MotorType.kBrushless);

  /** Creates a new Arm. */
  public Arm() {
    rMotor.follow(lMotor, true);
  }

  public void setArmSpeed(double speed) {
    lMotor.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
