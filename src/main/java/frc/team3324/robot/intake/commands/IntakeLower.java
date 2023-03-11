package frc.team3324.robot.intake.commands;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team3324.robot.intake.Intake;

public class IntakeLower extends CommandBase{
   Intake intake;
   double speed;

   public IntakeLower(Intake intake, double speed) {
       addRequirements(intake);
       this.intake = intake;
       this.speed = speed;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        intake.setLowerBrakeMode(IdleMode.kCoast);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        intake.setLowerIntakeSpeed(speed);
        if (speed < 0) {
            
        }
        else {
            
        }
    } 

    @Override
    public void end(boolean interrupted){
        intake.setLowerIntakeSpeed(0);
        intake.setLowerBrakeMode(IdleMode.kBrake);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
