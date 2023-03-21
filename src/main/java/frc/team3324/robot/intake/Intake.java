// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team3324.robot.intake;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team3324.robot.util.Constants;

public class Intake extends SubsystemBase {
  private static final CANSparkMax intakeMotorCube = Constants.Intake.INTAKE_MOTOR_CUBE;
  private static final CANSparkMax intakeMotorCone = Constants.Intake.INTAKE_MOTOR_CONE;
  
  public Intake() {}

  public void setCubeIntakeSpeed(double speed) {
    intakeMotorCube.set(speed);
  }
  public void setConeIntakeSpeed(double speed) {
    intakeMotorCone.set(speed);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }
}
