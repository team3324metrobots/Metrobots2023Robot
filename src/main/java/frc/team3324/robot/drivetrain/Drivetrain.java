// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team3324.robot.drivetrain;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team3324.robot.util.Constants;
import io.github.oblarg.oblog.annotations.Log;

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
  private final static CANSparkMax rfMotor = new CANSparkMax(Constants.Drivetrain.R_FRONT_MOTOR, MotorType.kBrushless);
  private final static CANSparkMax rmMotor = new CANSparkMax(Constants.Drivetrain.R_MIDDLE_MOTOR, MotorType.kBrushless);
  private final static CANSparkMax rbMotor = new CANSparkMax(Constants.Drivetrain.R_BACK_MOTOR, MotorType.kBrushless);
  // left side
  private final static CANSparkMax lfMotor = new CANSparkMax(Constants.Drivetrain.L_FRONT_MOTOR, MotorType.kBrushless);
  private final static CANSparkMax lmMotor = new CANSparkMax(Constants.Drivetrain.L_MIDDLE_MOTOR, MotorType.kBrushless);
  private final static CANSparkMax lbMotor = new CANSparkMax(Constants.Drivetrain.L_BACK_MOTOR, MotorType.kBrushless);

  // --- ENCODERS ---
  private static RelativeEncoder rEncoder = rmMotor.getEncoder();
  private static RelativeEncoder lEncoder = lmMotor.getEncoder();
  
  // --- GYRO ---
  private final AHRS navX = new AHRS(SPI.Port.kMXP);
  private DifferentialDriveOdometry driveOdometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(-1.0 * navX.getYaw()), lEncoder.getPosition(), rEncoder.getPosition());
  private DifferentialDriveKinematics driveKinematics = new DifferentialDriveKinematics(0.7112);

  // --- PID CONTROL ---
  private PIDController PIDControlYaw = new PIDController(0.008, 0.0001, 0.001);
  private PIDController PIDControlPitch = new PIDController(0, 0, 0);
  private SimpleMotorFeedforward FeedforwardDT = new SimpleMotorFeedforward(0.59019, 0.038769, 0.0049377);
 
  private static DifferentialDrive drive = new DifferentialDrive(lmMotor, rmMotor);

  private double currentVelocity = getVelocityMeters();
  private long currentTime = System.currentTimeMillis();
  
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

    // invert right-side motors because they're backwards :/
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
    navX.reset();

    // burn flash all changes made so they stick
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

  public void setOutputVolts(double lV, double rV) {
    lmMotor.setVoltage(lV);
    rmMotor.setVoltage(rV);
  }

  public void setPIDYawTolerance(double tolerance) {
    PIDControlYaw.setTolerance(tolerance);
  }

  public void setPIDPitchTolerance(double tolerance) {
    PIDControlPitch.setTolerance(tolerance);
  }

  public void setPitchPID(double kP, double kI, double kD) {
    PIDControlPitch.setP(kP);
    PIDControlPitch.setI(kI);
    PIDControlPitch.setD(kD);
  }

  public double getLeftEncoderPosition() {
    return lEncoder.getPosition();
  }

  public double getRightEncoderPosition() {
    return rEncoder.getPosition();
  }

  public double getPosition() {
    return (lEncoder.getPosition() - rEncoder.getPosition()) / 2;
  }

  public double getVelocity() {
    return (rEncoder.getVelocity() + lEncoder.getVelocity()) / 2;
  }

  public double getVelocityMeters() {
    return (getVelocity() * Constants.Drivetrain.CONVERSION_RATIO / 60) * 0.0254;
  }

  public double getAccelerationMeters() {
    return (getVelocityMeters() - currentVelocity) / (System.currentTimeMillis() - currentTime);
  }

  public AHRS getGyro() {
    return this.navX;
  }

  public double getGyroAngle() {
    return navX.getAngle();
  }

  public double getGyroAngle360() {
    // this function is for viewing the gyro's current angle in a human-viewable way
    if (navX.getAngle() > 360) {
      return navX.getAngle() - 360;
    }
    else if (navX.getAngle() < 0) {
      return navX.getAngle() + 360;
    }
    else {
      return navX.getAngle();
    }
  }

  public double getGyroPitch() {
    return navX.getPitch();
  }

  public double getGyroYaw() {
    return navX.getYaw();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(lEncoder.getVelocity(), rEncoder.getVelocity());
  }

  public Pose2d getPose() {
    // like getPosition() but uses the robot odometry rather than the encoder positions alone
    return driveOdometry.getPoseMeters();
  }

  public DifferentialDriveKinematics getKinematics() {
    return this.driveKinematics;
  }

  public DifferentialDriveOdometry getOdometry() {
    return this.driveOdometry;
  }

  @Log
  public double getDistance() {
    return this.getPosition() * Constants.Drivetrain.CIRCUMFERENCE_METERS;
  }

  public PIDController getPIDYaw() {
    return this.PIDControlYaw;
  }

  public double getPIDYawSpeed(double setpoint) {
    return PIDControlYaw.calculate(getGyroYaw(), setpoint);
  }

  public double getPIDPitchSpeed(double setpoint) {
    return PIDControlPitch.calculate(getGyroPitch(), setpoint);
  }

  public boolean pitchAtSetpoint() {
    return PIDControlPitch.atSetpoint();
  }

  public SimpleMotorFeedforward getFeedforward() {
    return this.FeedforwardDT;
  }

  public void resetEncoders() {
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
    SmartDashboard.putNumber("Velocity", getVelocityMeters());
    SmartDashboard.putNumber("Acceleration", getAccelerationMeters());

    SmartDashboard.putNumber("Robot Pitch", getGyroPitch());
    SmartDashboard.putNumber("Robot Yaw", getGyroYaw());
    SmartDashboard.putNumber("Robot Angle", getGyroAngle360());

    SmartDashboard.putNumber("Robot X Position", getPose().getX());
    SmartDashboard.putNumber("Robot Y Position", getPose().getY());

    currentVelocity = getVelocityMeters();
    currentTime = System.currentTimeMillis();

    driveOdometry.update(Rotation2d.fromDegrees(getGyroAngle()), getWheelSpeeds().leftMetersPerSecond, getWheelSpeeds().rightMetersPerSecond);
  }
}
