// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team3324.robot.util;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.team3324.library.motorcontrollers.SmartMotionSparkMAX;

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
    public static final CANSparkMax INTAKE_MOTOR_CUBE = new CANSparkMax(11, MotorType.kBrushless);
    public static final CANSparkMax INTAKE_MOTOR_CONE = new CANSparkMax(12, MotorType.kBrushless);
  }

  public static class Drivetrain {
    // --- ENCODER AND AUTO CONSTANTS ---
    // gear ratio
    public static final double DT_GEAR_RATIO = 24.0 / 50.0;

    public static final double CONVERSION_RATIO = 2.16;

    public static final double WHEEL_DIAMETER_METERS = 6.125 / 39.36;
    public static final double CIRCUMFERENCE_METERS = Math.PI * WHEEL_DIAMETER_METERS;
    
    // --- PID ---
    public static final double SmartMotion_P = 0.008;
    public static final double SmartMotion_I = 0;
    public static final double SmartMotion_D = 0.001;
    public static final double SmartMotion_F = 0.38769;
    public static final double DriveStraight_P = 0.28;
    public static final double DriveStraight_I = 0.03;
    public static final double DriveStraight_D = 0.0;
 
    // --- DRIVETRAIN MOTORS ---
    // right side
    public final static SmartMotionSparkMAX RIGHT_FRONT_MOTOR = new SmartMotionSparkMAX(7, MotorType.kBrushless, 40, 0.25, SmartMotion_P, SmartMotion_I, SmartMotion_D, SmartMotion_F);
    public final static SmartMotionSparkMAX RIGHT_MIDDLE_MOTOR = new SmartMotionSparkMAX(6, MotorType.kBrushless, 40, 0.25, SmartMotion_P, SmartMotion_I, SmartMotion_D, SmartMotion_F );
    public final static SmartMotionSparkMAX RIGHT_BACK_MOTOR = new SmartMotionSparkMAX(5, MotorType.kBrushless, 40, 0.25, SmartMotion_P, SmartMotion_I, SmartMotion_D, SmartMotion_F);
    // left side
    public final static SmartMotionSparkMAX LEFT_FRONT_MOTOR = new SmartMotionSparkMAX(4, MotorType.kBrushless, 40, 0.25, SmartMotion_P, SmartMotion_I, SmartMotion_D, SmartMotion_F);
    public final static SmartMotionSparkMAX LEFT_MIDDLE_MOTOR = new SmartMotionSparkMAX(3, MotorType.kBrushless, 40, 0.25, SmartMotion_P, SmartMotion_I, SmartMotion_D, SmartMotion_F);
    public final static SmartMotionSparkMAX LEFT_BACK_MOTOR = new SmartMotionSparkMAX(2, MotorType.kBrushless, 40, 0.25, SmartMotion_P, SmartMotion_I, SmartMotion_D, SmartMotion_F);
    
    public static final double CONTROLLER_DEADZONE = 0.12;
  }
}
