// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team3324.robot.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.team3324.robot.util.Constants;
import frc.team6300.NorthwoodDrivers.LoggedNeo;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
  // --- ARM MOTORS ---
  private final LoggedNeo lMotor = new LoggedNeo(Constants.Arm.ARM_MOTOR_L, false, 30, 1/180.0 );
  private final LoggedNeo rMotor = new LoggedNeo(Constants.Arm.ARM_MOTOR_R, true, 30, 1/180.0 );
  private final LoggedNeo teleMotor = new LoggedNeo(Constants.Arm.TELESCOPE_MOTOR, false, 30, 1/180.0 ); //FIXME gear ratio on tele Motor



  // --- ARM FEEDFORWARD CONTROLLER ---
  private double kS;
  private double kG;
  private double kV;
  private ArmFeedforward FeedforwardArm = new ArmFeedforward(kS, kG, kV);

  /** Creates a new Arm. */
  public Arm() {
    rMotor.setSlave(lMotor.getMotorObject());


  }

  public enum ArmPreset {
    INTAKE,
    UP,
    HIGH,
    MID,
    LOW
  } 

  // --- GETTERS & SETTERS ---

  public void setArmSpeed(double speed) {
    lMotor.setPercentOutput(speed);
  }

  public void setTeleSpeed(double speed) {
    teleMotor.setPercentOutput(speed);
  }

  public double getArmPosition() {
    return lMotor.getPosition();
  }

  public double getTelePosition() {
    return teleMotor.getPosition();
  }

  public double getArmVelocity() {
    return lMotor.getVelocity();
  }

  public double getFeedForwardSpeed() {
    return FeedforwardArm.calculate(getArmPosition(), getArmVelocity());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
