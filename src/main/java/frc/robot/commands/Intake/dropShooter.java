package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class dropShooter extends CommandBase {
    
    public dropShooter() {
        addRequirements(Robot.Gun);
    }

    public void initialize() {

    }

    public void execute() {
        Robot.Gun.rotate(-.475);
    }

    public boolean isFinished() {
        return super.isFinished();
    }

    public void end(boolean interrupted) {

    }

}
