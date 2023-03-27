// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team3324.robot;

import frc.team3324.robot.arm.Arm;
import frc.team3324.robot.arm.Arm.ArmPreset;
import frc.team3324.robot.arm.commands.AutoArm;
import frc.team3324.robot.arm.commands.ControlArm;
import frc.team3324.robot.arm.commands.TelescopeArm;
import frc.team3324.robot.auto.ScoreTwoGetOneBalance;
import frc.team3324.robot.auto.commands.ScoreCubeHigh;
import frc.team3324.robot.drivetrain.Drivetrain;
import frc.team3324.robot.drivetrain.commands.AutoBalance;
import frc.team3324.robot.drivetrain.commands.DebugDrivetrain;
import frc.team3324.robot.drivetrain.commands.Drive;
import frc.team3324.robot.intake.Intake;
import frc.team3324.robot.intake.commands.IntakeCone;
import frc.team3324.robot.intake.commands.IntakeCube;
import frc.team3324.robot.vision.Vision;
import frc.team3324.robot.vision.commands.AlignWithVision;
import frc.team3324.robot.vision.commands.MoveArmWithVision;

import java.util.HashMap;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
// import frc.team3324.robot.arm.commands.TelescopeArm;
// import edu.wpi.first.networktables.NetworkTableInstance;
// import frc.team3324.robot.drivetrain.commands.AutoBalance;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */

public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  // --- INITIALIZE DRIVER CONTROLLERS ---
  private final static CommandXboxController primaryDriver = new CommandXboxController(0);
  private final static CommandXboxController secondaryDriver = new CommandXboxController(1);

  // --- INITIALIZE SUBSYSTEMS ---
  private static Arm arm = new Arm();
  private static Drivetrain drivetrain = new Drivetrain();
  private static Intake intake = new Intake();
  private static Vision vision = new Vision();

  // --- PATHPLANNER AUTO EVENT MAP ---
  public static HashMap<String, Command> eventMap = new HashMap<>();

  // private NetworkTableInstance nt_instance = NetworkTableInstance.getDefault();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
   
    eventMap.putIfAbsent("ScoreCubeHigh", new ScoreCubeHigh(arm, intake));
    eventMap.putIfAbsent("IntakeCube", new IntakeCube(intake, 0));
    eventMap.putIfAbsent("ScoreCubeMid", new ScoreCubeHigh(arm, intake));
    eventMap.putIfAbsent("AutoBalance", new AutoBalance(drivetrain));
  }

  private void configureBindings() {
    // --- DEFAULT COMMANDS ---
    drivetrain.setDefaultCommand(new Drive(drivetrain, primaryDriver::getLeftTriggerAxis, primaryDriver::getRightTriggerAxis, primaryDriver::getLeftX));
    arm.setDefaultCommand(new ControlArm(arm, secondaryDriver::getLeftY));

    // --- ARM PRESETS ---
    secondaryDriver.povDown().whileTrue(new AutoArm(arm, ArmPreset.HYBRID));
    secondaryDriver.povLeft().whileTrue(new AutoArm(arm, ArmPreset.NONHYBRID));
    secondaryDriver.povUp().whileTrue(new AutoArm(arm, ArmPreset.UP));
    secondaryDriver.povRight().whileTrue(new AutoArm(arm, ArmPreset.INTAKE));

    // --- VISION COMMANDS ---
    primaryDriver.b().whileTrue(new AlignWithVision(vision, drivetrain));
    primaryDriver.a().whileTrue(new MoveArmWithVision(vision, drivetrain, arm));

    // --- INTAKE COMMANDS --- 
    secondaryDriver.rightTrigger().whileTrue(new IntakeCone(intake, 0.2));
    secondaryDriver.leftTrigger().whileTrue(new IntakeCone(intake, -0.2));
    secondaryDriver.leftBumper().whileTrue(new IntakeCube(intake, -1.0));
    secondaryDriver.rightBumper().whileTrue(new IntakeCube(intake, 0.45));

    // --- TELESCOPE COMMANDS ---
    secondaryDriver.b().whileTrue(new TelescopeArm(arm, 1.0));
    secondaryDriver.a().whileTrue(new TelescopeArm(arm, -1.0));

    primaryDriver.y().whileTrue(new DebugDrivetrain(drivetrain));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return new ScoreTwoGetOneBalance(drivetrain);
  }
}
