// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team3324.robot.util;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.util.Units;
import frc.team3324.library.motorcontrollers.SmartMotionSparkMAX;
import frc.team6300.NorthwoodDrivers.LoggedNeo;

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
  public static final boolean tuningMode = true;
  public static double loopPeriodSeconds = 0.02;
  public static class Arm {
    public static final double ARM_CONTROLLER_DEADZONE = 0.12;
    public static final double ARM_GEAR_RATIO = 1 / 180.0;

    public static final SmartMotionSparkMAX ARM_MOTOR_L = new SmartMotionSparkMAX(8, false, 40, ARM_GEAR_RATIO, 0.25, 0.19167, 0, 0, 0.11957);
    public static final SmartMotionSparkMAX ARM_MOTOR_R = new SmartMotionSparkMAX(9, true, 40, ARM_GEAR_RATIO, 0.25, 0.19167, 0, 0, 0.11957);
    public static final LoggedNeo TELESCOPE_MOTOR = new LoggedNeo(10, false, 40, 0); // TODO: find gear ratio of telescope
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

    public static final double TrackWidth = Units.inchesToMeters(33);
    
    // --- RAMSETE D and Zeta Values -- 
    // find with SysID
    public static final double ramseteD = 0.0; 
    public static final double ramseteZ = 0.0;

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
    public final static LoggedNeo RIGHT_FRONT_MOTOR = new LoggedNeo(7, true, 40, DT_GEAR_RATIO*CIRCUMFERENCE_METERS);
    public final static LoggedNeo RIGHT_MIDDLE_MOTOR = new LoggedNeo(6, true, 40, DT_GEAR_RATIO*CIRCUMFERENCE_METERS);
    public final static LoggedNeo RIGHT_BACK_MOTOR = new LoggedNeo(5, true, 40, DT_GEAR_RATIO*CIRCUMFERENCE_METERS);
   
    // left side
    public final static LoggedNeo LEFT_FRONT_MOTOR = new LoggedNeo(4, false, 40, DT_GEAR_RATIO*CIRCUMFERENCE_METERS);
    public final static LoggedNeo LEFT_MIDDLE_MOTOR = new LoggedNeo(3, false, 40, DT_GEAR_RATIO*CIRCUMFERENCE_METERS);
    public final static LoggedNeo LEFT_BACK_MOTOR = new LoggedNeo(2, false, 40, DT_GEAR_RATIO*CIRCUMFERENCE_METERS);
    public static final double CONTROLLER_DEADZONE = 0.12;
  }
}
