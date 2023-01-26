// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

/**
 * ------ CAN ID KEY ------
 * 2-7: Drivetrain
 * 8-10: Arm
 *  - 8,9: Control arm
 *  - 10: Telescope arm
 * 11+: Other subsystems
 */
public final class Constants {
  public static class Arm {
    public static final int ARM_MOTOR_L = 8;
    public static final int ARM_MOTOR_R = 9;
    public static final int TELESCOPE_MOTOR = 10;
  }

  public static class Drivetrain {
    // --- ENCODER AND AUTO CONSTANTS ---
    // gear ratios
    public static final double DT_GEAR_RATIO_STAGE1 = 12.0 / 50.0;
    public static final double DT_GEAR_RATIO_STAGE2 = 20.0 / 54.0;

    public static final double CONVERSION_RATIO = DT_GEAR_RATIO_STAGE1 * DT_GEAR_RATIO_STAGE2;
    public static final double WHEEL_DIAMETER_METERS = 6.125 / 39.36;
    public static final double CIRCUMFERENCE_METERS = Math.PI * WHEEL_DIAMETER_METERS;

    // --- DRIVETRAIN MOTORS ---
    // right side
    public static final int R_FRONT_MOTOR = 7;
    public static final int R_MIDDLE_MOTOR = 6;
    public static final int R_BACK_MOTOR = 5;

    // left side
    public static final int L_FRONT_MOTOR = 4;
    public static final int L_MIDDLE_MOTOR = 3;
    public static final int L_BACK_MOTOR = 2;
  }
}
