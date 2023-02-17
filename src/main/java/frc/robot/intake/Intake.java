// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Constants;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private static final CANSparkMax intakeMotorCube = new CANSparkMax(Constants.Intake.INTAKE_MOTOR_CUBE, MotorType.kBrushless);
  private static final CANSparkMax intakeMotorCone = new CANSparkMax(Constants.Intake.INTAKE_MOTOR_CONE, MotorType.kBrushless);

  public enum IntakeMotor {
    CubeIntake(intakeMotorCube),
    ConeIntake(intakeMotorCone);

    private final CANSparkMax motor;

    IntakeMotor(CANSparkMax motor) {
      this.motor = motor;
    }
    public void set(double speed) {
      motor.set(speed);
    }
  }
  

  public Intake() {}

  public void setIntakeSpeed(IntakeMotor motor, double speed) {
    motor.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
