// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team3324.robot.util;

/**
 * ------ CAN ID KEY ------
 * 2-7: Drivetrain
 *  - 2-4: Left side
 *   - 2: Left back motor
 *   - 3: Left middle motor
 *   - 4: Left front motor
 *  - 5-7: Right side
 *   - 5: Right back motor
 *   - 6: Right middle motor
 *   - 7: Right front motor
 * 8-10: Arm
 *  - 8, 9: Control arm
 *   - 8: Left arm motor
 *   - 9: Right arm motor
 *  - 10: Telescope arm
 * 11-13: Intake
 *  - 11: Cube intake
 *  - 12: Left Cone intake
 *  - 13: Right Cone intake
 */
public final class Constants {
  public static class Arm {
    public static final int ARM_MOTOR_L = 8;
    public static final int ARM_MOTOR_R = 9;
    public static final int TELESCOPE_MOTOR = 10;

    public static final double ARM_CONTROLLER_DEADZONE = 0.5;
  }

  public static class Intake {
    public static final int INTAKE_MOTOR_CUBE = 11;
    public static final int INTAKE_MOTOR_CONE_LEFT = 12;
    public static final int INTAKE_MOTOR_CONE_RIGHT = 13;
  }

  public static class Drivetrain {
    // --- ENCODER AND AUTO CONSTANTS ---
    // gear ratio
    public static final double DT_GEAR_RATIO = 24.0 / 50.0;

    public static final double CONVERSION_RATIO = 2.16;

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

    public static final double CONTROLLER_DEADZONE = 0.12;

    // --- PID ---
    public static final double DriveStraight_P = 0.28;
    public static final double DriveStraight_I = 0.03;
    public static final double DriveStraight_D = 0.0;
  }
}
