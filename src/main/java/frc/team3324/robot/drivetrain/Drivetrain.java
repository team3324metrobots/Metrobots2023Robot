// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team3324.robot.drivetrain;



import org.littletonrobotics.junction.Logger;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPRamseteCommand;


import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team3324.robot.util.Constants;
import frc.team6300.NorthwoodDrivers.LoggedMotorIOInputsAutoLogged;
import frc.team6300.NorthwoodDrivers.LoggedNeo;


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

  private static LoggedNeo rfMotor = Constants.Drivetrain.RIGHT_FRONT_MOTOR;
  private static LoggedNeo rmMotor = Constants.Drivetrain.RIGHT_MIDDLE_MOTOR;
  private static LoggedNeo rbMotor = Constants.Drivetrain.RIGHT_BACK_MOTOR;
  private static LoggedNeo lfMotor = Constants.Drivetrain.LEFT_FRONT_MOTOR;
  private static LoggedNeo lmMotor = Constants.Drivetrain.LEFT_MIDDLE_MOTOR;
  private static LoggedNeo lbMotor = Constants.Drivetrain.LEFT_BACK_MOTOR;

  private static LoggedMotorIOInputsAutoLogged rmMotorLog = new LoggedMotorIOInputsAutoLogged();
  private static LoggedMotorIOInputsAutoLogged rfMotorLog = new LoggedMotorIOInputsAutoLogged();
  private static LoggedMotorIOInputsAutoLogged rbMotorLog = new LoggedMotorIOInputsAutoLogged();
  private static LoggedMotorIOInputsAutoLogged lmMotorLog = new LoggedMotorIOInputsAutoLogged();
  private static LoggedMotorIOInputsAutoLogged lfMotorLog = new LoggedMotorIOInputsAutoLogged();
  private static LoggedMotorIOInputsAutoLogged lbMotorLog = new LoggedMotorIOInputsAutoLogged();

  // --- ENCODERS ---
 

  
  // --- GYRO ---
  private final AHRS navX = new AHRS(SPI.Port.kMXP);
  private DifferentialDrivePoseEstimator drivePoseEstimator;
  private DifferentialDriveOdometry driveOdometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(-1.0 * navX.getYaw()), lmMotor.getPosition(), rmMotor.getPosition());
  private DifferentialDriveKinematics driveKinematics = new DifferentialDriveKinematics(0.7112);

  // --- PID CONTROL ---
  private PIDController PIDControlYaw = new PIDController(0.008, 0.0001, 0.001);
  private PIDController PIDControlPitch = new PIDController(0, 0, 0);
  private SimpleMotorFeedforward FeedforwardDT = new SimpleMotorFeedforward(0.59019, 0.038769, 0.0049377);
 
  private static DifferentialDrive drive = new DifferentialDrive(lmMotor.getMotorObject(), rmMotor.getMotorObject());

  


  private long currentTime = System.currentTimeMillis();
  
  /** Creates a new Drivetrain. */
  public Drivetrain() {
    // reset motors so we can work with them later
   

    // set followers
    rfMotor.setSlave(rmMotor.getMotorObject());
    rbMotor.setSlave(rmMotor.getMotorObject());

    lfMotor.setSlave(lmMotor.getMotorObject());
    lbMotor.setSlave(lmMotor.getMotorObject()); 

    // invert right-side motors because they're backwards :/
    
    // ramp rate (BE CAREFUL WHEN CHANGING)
    rfMotor.configureRampRate(0.25);
    rbMotor.configureRampRate(0.25);
    rmMotor.configureRampRate(0.25);
    lfMotor.configureRampRate(0.25);
    lmMotor.configureRampRate(0.25);
    lbMotor.configureRampRate(0.25);

    
    resetEncoders();
    navX.reset();

   
    var nativeCovMatrix = VecBuilder.fill(0.02, 0.02, 0.01);
    
    var visionCovMatrix = VecBuilder.fill(0.1, 0.1, 0.1);
   
    drive.setSafetyEnabled(true);
    drivePoseEstimator = new DifferentialDrivePoseEstimator(
      getKinematics(), 
      Rotation2d.fromDegrees(-getGyroAngle()), 
      lmMotor.getPosition(), 
      rmMotor.getPosition(), 
      new Pose2d(), 
      nativeCovMatrix, 
      visionCovMatrix);

  }

  // --- GETTERS & SETTERS ---
 

  public void setMaxOutput(double speed) {
    drive.setMaxOutput(speed);
  }

  public void setOutputVolts(double lV, double rV) {
    lmMotor.setPercentOutput(lV/12);
    rmMotor.setPercentOutput(rV/12);
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
    return lmMotor.getPosition();
  }

  public double getRightEncoderPosition() {
    return rmMotor.getPosition();
  }
/* this will not return the robot's "local" velocity or position, because the position and local velocity are influenced, defined and derived differentially.
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
  }*/

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
  public void acceptWheelSpeeds(double leftVelocity, double rightVelocity){
    lmMotor.setVelocity(leftVelocity, 0, 0);
    lmMotor.setVelocity(rightVelocity, 0, 0);

  } 

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(lmMotor.getVelocity(), rmMotor.getVelocity());
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
  public void setKnownPose(Pose2d knownPose){
    driveOdometry.resetPosition(Rotation2d.fromDegrees(-getGyroAngle()), lbMotor.getPosition(), rmMotor.getPosition(), knownPose);
  }

  public Command  FollowPath(PathPlannerTrajectory path, Drivetrain m_drive){
        PPRamseteCommand pathFollowCommand = 

        new PPRamseteCommand( 
            path, 

            m_drive:: getPose,

            new RamseteController(
              Constants.Drivetrain.ramseteD,
              Constants.Drivetrain.ramseteZ), 

            m_drive.getKinematics(), 

            m_drive:: acceptWheelSpeeds, 

            true,

            m_drive
            );
        return pathFollowCommand;
  }

  /* will be redundant after odometry is properly implemented
  @Log
  public double getDistance() {
    return this.getPosition() * Constants.Drivetrain.CIRCUMFERENCE_METERS;
  }*/

  public double getDistance(){
    return driveKinematics.toTwist2d(lmMotor.getPosition(), rmMotor.getPosition()).dx;
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
    lmMotor.setEncoder(0.0);
    rmMotor.setEncoder(0.0);
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
  
    SmartDashboard.putNumber("Robot Pitch", getGyroPitch());
    SmartDashboard.putNumber("Robot Yaw", getGyroYaw());
    SmartDashboard.putNumber("Robot Angle", getGyroAngle360());

    SmartDashboard.putNumber("Robot X Position", getPose().getX());
    SmartDashboard.putNumber("Robot Y Position", getPose().getY());

    Logger.getInstance().recordOutput("RobotPose", getPose());

    rmMotor.updateInputs(rmMotorLog);
    rfMotor.updateInputs(rfMotorLog);
    rbMotor.updateInputs(rbMotorLog);
    lmMotor.updateInputs(lmMotorLog);
    lbMotor.updateInputs(lbMotorLog);
    lfMotor.updateInputs(lfMotorLog);

    currentTime = System.currentTimeMillis();

    driveOdometry.update(Rotation2d.fromDegrees(getGyroAngle()), getWheelSpeeds().leftMetersPerSecond, getWheelSpeeds().rightMetersPerSecond);
    drivePoseEstimator.update(Rotation2d.fromDegrees(getGyroAngle()), getWheelSpeeds().leftMetersPerSecond, getWheelSpeeds().rightMetersPerSecond);

  }
}
