package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class intakeDTFSlow extends CommandBase {
    
    double duration;
    int frameCount;

    public intakeDTFSlow(double duration) {
        this.duration = duration;
        addRequirements(Robot.Spintake);
    }

    public void initialize() {
        frameCount = 0;
    }

    public void execute() {
        Robot.Spintake.spin(.6, .6);
        frameCount++;
    }

    public boolean isFinished() {
        return super.isFinished() || frameCount > duration;
    }

    public void end() {
        Robot.Spintake.spin(0, 0);
    }
}
