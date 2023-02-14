// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.arm.Arm;
import frc.robot.arm.commands.ControlArm;
import frc.robot.drivetrain.Drivetrain;
import frc.robot.drivetrain.commands.AutoBalance;
import frc.robot.drivetrain.commands.Drive;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  // --- INITIALIZE DRIVER CONTROLLERS ---
  public final static CommandXboxController primaryDriver = new CommandXboxController(0);
  public final static CommandXboxController secondaryDriver = new CommandXboxController(1);

  // --- PNEUMATICS ---
  private final Compressor compressor = new Compressor(PneumaticsModuleType.CTREPCM);
  private final DoubleSolenoid solenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 6, 7); 

  // --- INITIALIZE SUBSYSTEMS ---
  public static Arm arm = new Arm();
  public static Drivetrain drivetrain = new Drivetrain();

  private NetworkTableInstance nt_instance = NetworkTableInstance.getDefault();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    drivetrain.setDefaultCommand(new Drive(drivetrain, primaryDriver::getLeftX));
    arm.setDefaultCommand(new ControlArm(arm, primaryDriver::getRightY));
    primaryDriver.y().whileTrue(new AutoBalance(drivetrain));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
}
