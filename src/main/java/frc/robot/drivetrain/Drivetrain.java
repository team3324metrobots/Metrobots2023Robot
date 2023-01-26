// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drivetrain;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Constants;

public class Drivetrain extends SubsystemBase {
  // --- DRIVETRAIN MOTORS ---
  /*
   * rfMotor = right front motor
   * rmMotor = right middle motor
   * rbMotor = right back motor
   * lfMotor = left front motor
   * lmMotor = left middle motor
   * lbMotor = left back motor
   */
  // right side
  private final CANSparkMax rfMotor = new CANSparkMax(Constants.Drivetrain.R_FRONT_MOTOR, MotorType.kBrushless);
  private final static CANSparkMax rmMotor = new CANSparkMax(Constants.Drivetrain.R_MIDDLE_MOTOR, MotorType.kBrushless);
  private final CANSparkMax rbMotor = new CANSparkMax(Constants.Drivetrain.R_BACK_MOTOR, MotorType.kBrushless);
  // left side
  private final CANSparkMax lfMotor = new CANSparkMax(Constants.Drivetrain.L_FRONT_MOTOR, MotorType.kBrushless);
  private final static CANSparkMax lmMotor = new CANSparkMax(Constants.Drivetrain.L_MIDDLE_MOTOR, MotorType.kBrushless);
  private final CANSparkMax lbMotor = new CANSparkMax(Constants.Drivetrain.L_BACK_MOTOR, MotorType.kBrushless);

  // --- ENCODERS ---
  private static RelativeEncoder rEncoder = rmMotor.getEncoder();
  private static RelativeEncoder lEncoder = lmMotor.getEncoder();

  // --- GYRO ---
  private final AHRS navX = new AHRS(SPI.Port.kMXP);
  private DifferentialDriveOdometry driveOdometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(-1.0 * navX.getYaw()), lEncoder.getPosition(), rEncoder.getPosition());

 
  private DifferentialDrive drive = new DifferentialDrive(lmMotor, rmMotor);
  
  /** Creates a new Drivetrain. */
  public Drivetrain() {
    // reset motors so we can work with them later
    rfMotor.restoreFactoryDefaults();
    rmMotor.restoreFactoryDefaults();
    rbMotor.restoreFactoryDefaults();
    lfMotor.restoreFactoryDefaults();
    lmMotor.restoreFactoryDefaults();
    lbMotor.restoreFactoryDefaults();

    // set followers
    rfMotor.follow(rmMotor, false);
    rbMotor.follow(rmMotor, false);

    lfMotor.follow(lmMotor, false);
    lbMotor.follow(lmMotor, false); 

    // invert motors question mark?
    rfMotor.setInverted(true);
    rmMotor.setInverted(true);
    rbMotor.setInverted(true);

    // ramp rate (BE CAREFUL WHEN CHANGING)
    rmMotor.setOpenLoopRampRate(0.25);
    lmMotor.setOpenLoopRampRate(0.25);

    // current limits (DO NOT CHANGE NO MATTER WHAT UNLESS YOU WANT A SMOKING NEO)
    rmMotor.setSmartCurrentLimit(40);
    lmMotor.setSmartCurrentLimit(40);
    rmMotor.setSecondaryCurrentLimit(40.0);
    lmMotor.setSecondaryCurrentLimit(40.0);

    setBrakeMode(IdleMode.kBrake);
    resetEncoders();

    rfMotor.burnFlash();
    rmMotor.burnFlash();
    rbMotor.burnFlash();
    lfMotor.burnFlash();
    lmMotor.burnFlash();
    lbMotor.burnFlash();

    drive.setSafetyEnabled(true);
  }

  // --- GETTERS & SETTERS ---
  public void setBrakeMode(CANSparkMax.IdleMode brakeMode) {
    rfMotor.setIdleMode(brakeMode);
    rmMotor.setIdleMode(brakeMode);
    rbMotor.setIdleMode(brakeMode);
    lfMotor.setIdleMode(brakeMode);
    lmMotor.setIdleMode(brakeMode);
    lbMotor.setIdleMode(brakeMode);
  }

  public void setMaxOutput(double speed) {
    drive.setMaxOutput(speed);
  }

  public void getLeftEncoderPosition() {
    lEncoder.getPosition();
  }

  public void getRightEncoderPosition() {
    rEncoder.getPosition();
  }

  public double getVelocity() {
    return (lEncoder.getVelocity() - rEncoder.getVelocity()) / 2;
  }

  public void getGyroPitch() {
    navX.getPitch();
  }

  public void getGyroYaw() {
    navX.getYaw();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(lEncoder.getVelocity(), rEncoder.getVelocity());
  }

  public double getDistance() {
    return this.getDistance() * Constants.Drivetrain.CIRCUMFERENCE_METERS;
  }

  public static void resetEncoders() {
    lEncoder.setPosition(0.0);
    rEncoder.setPosition(0.0);
  }

  private void curvatureDrive(double xSpeed, double ySpeed, boolean quickTurn) {
    drive.curvatureDrive(xSpeed, ySpeed, quickTurn);
  }

  public void curvatureDrive(double xSpeed, double ySpeed) {
    curvatureDrive(xSpeed, ySpeed, true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Left Wheel Speed", getWheelSpeeds().leftMetersPerSecond);
    SmartDashboard.putNumber("Right Wheel Speed", getWheelSpeeds().rightMetersPerSecond);
    SmartDashboard.putNumber("Velocity", getVelocity());
  }
}
