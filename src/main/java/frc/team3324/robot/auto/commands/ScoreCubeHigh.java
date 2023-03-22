// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team3324.robot.auto.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team3324.robot.arm.commands.AutoArm;
import frc.team3324.robot.arm.commands.AutoTelescope;
import frc.team3324.robot.intake.commands.IntakeCube;
import frc.team3324.robot.arm.Arm;
import frc.team3324.robot.arm.Arm.ArmPreset;
import frc.team3324.robot.arm.Arm.TelePreset;
import frc.team3324.robot.intake.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreCubeHigh extends SequentialCommandGroup {
  /** Creates a new ScoreCube. */
  public ScoreCubeHigh(Arm arm, Intake intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new AutoArm(arm, ArmPreset.NONHYBRID),
      new AutoTelescope(arm, TelePreset.STAGE2),
      new IntakeCube(intake, -1.0),
      new AutoTelescope(arm, TelePreset.STAGE1)
    );
  }
}
