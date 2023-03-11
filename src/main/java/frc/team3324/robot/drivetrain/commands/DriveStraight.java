package frc.team3324.robot.drivetrain.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team3324.robot.drivetrain.Drivetrain;
import frc.team3324.robot.util.Constants;

public class DriveStraight extends CommandBase {
    Drivetrain drivetrain;
    double distance;

    PIDController distanceController;

    public DriveStraight(Drivetrain drivetrain, double distance) {
        addRequirements(drivetrain);

        this.drivetrain = drivetrain;
        this.distance = distance;
    }

    @Override
    public void initialize() {
        double goal = drivetrain.getDistance() + distance;
        SmartDashboard.putNumber("DriveStraight Goal", goal);

        this.distanceController = new PIDController(
            Constants.Drivetrain.DriveStraight_P,
            Constants.Drivetrain.DriveStraight_I,
            Constants.Drivetrain.DriveStraight_D
        );

        // this.distanceController = new PIDController(
        //     SmartDashboard.getNumber("Distance P", 0.0),
        //     SmartDashboard.getNumber("Distance I", 0.0),
        //     SmartDashboard.getNumber("Distance D", 0.0)
        // );

        this.distanceController.setSetpoint(goal);

        drivetrain.setMaxOutput(0.55);
    }

    @Override
    public void execute() {
        double speed = distanceController.calculate(drivetrain.getDistance());

        SmartDashboard.putNumber("Distance Speed", speed);

        drivetrain.curvatureDrive(0.0, (-1.0 * speed));
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setMaxOutput(1.0);
    }

    @Override
    public boolean isFinished() {
        return (distanceController.atSetpoint());
    }
}