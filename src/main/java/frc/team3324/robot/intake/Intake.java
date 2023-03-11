// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team3324.robot.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team3324.robot.util.Constants;

public class Intake extends SubsystemBase {
  private static final CANSparkMax intakeMotorCube = new CANSparkMax(Constants.Intake.INTAKE_MOTOR_CUBE, MotorType.kBrushless);
  private static final CANSparkMax intakeMotorConeLeft = new CANSparkMax(Constants.Intake.INTAKE_MOTOR_CONE_LEFT, MotorType.kBrushless);
  private static final CANSparkMax intakeMotorConeRight = new CANSparkMax(Constants.Intake.INTAKE_MOTOR_CONE_RIGHT, MotorType.kBrushless);
  private static final CANSparkMax intakeMotorLower = new CANSparkMax(Constants.Intake.INTAKE_MOTOR_LOWER, MotorType.kBrushless);
  private static final RelativeEncoder coneEncoderLeft = intakeMotorConeLeft.getEncoder();
  private static final RelativeEncoder coneEncoderRight = intakeMotorConeRight.getEncoder();
  
  public Intake() { 
    coneEncoderLeft.setPosition(0.0);
    coneEncoderRight.setPosition(0.0);
  }

  public void setCubeIntakeSpeed(double speed) {
    intakeMotorCube.set(speed);
  }

  public void setLowerIntakeSpeed(double speed) {
    intakeMotorLower.set(speed);
  }

  public void setLowerBrakeMode(IdleMode brakeMode) {
    intakeMotorLower.setIdleMode(brakeMode);
  }

  public void setConeIntakeSpeed(double speed) {
    intakeMotorConeLeft.set(speed);
    intakeMotorConeRight.set(-speed * 1.5);
  }

  public double getPosition() {
    coneEncoderLeft.getPosition();
    return coneEncoderRight.getPosition();
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
