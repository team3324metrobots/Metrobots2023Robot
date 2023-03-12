// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team3324.robot.vision.commands;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team3324.robot.arm.Arm;
import frc.team3324.robot.drivetrain.Drivetrain;
import frc.team3324.robot.util.Constants;
import frc.team3324.robot.vision.Vision;

public class MoveArmWithVision extends CommandBase {  

  Vision vision;
  Drivetrain drivetrain;
  Arm arm;

  // static final double kP = 0.1; // Proportional gain for PID control
  // static final double kI = 0.0; // Integral gain for PID control
  // static final double kF = 0.0; // Feedforward gain for PID control
  // static final double kToleranceDegrees = 1.0; // Tolerance for PID error in degrees

  /** Creates a new MoveArmWithVision. */
  public MoveArmWithVision(Vision vision, Drivetrain drivetrain, Arm arm) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(vision, drivetrain, arm);
    this.drivetrain = drivetrain;
    this.vision = vision;
    this.arm = arm;
  }

  final CANSparkMax rMotor = new CANSparkMax(Constants.Arm.ARM_MOTOR_R, MotorType.kBrushless);
  final CANSparkMax teleMotor = new CANSparkMax(Constants.Arm.TELESCOPE_MOTOR, MotorType.kBrushless);
  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {


  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0.0);
    // double error = -tx; // Negative sign to account for mirror image effect
    // double output = kP * error; // Proportional term

    // if (Math.abs(error) < kToleranceDegrees) {
    //   output = 0.0; // Within tolerance, don't rotate the arm

    //   lMotor.set(output); // Send the output to the rotation motor

      double targetAngle = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0.0);
      double targetDistance = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0.0);

      // Use proportional control to rotate the arm towards the target
      double rotationSpeed = 0.5 * targetAngle;
      rMotor.set(rotationSpeed);


      // Use proportional control to extend or contract the arm towards the target
      double extensionSpeed = 0.1 * targetDistance;
      teleMotor.set(extensionSpeed);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {



  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
