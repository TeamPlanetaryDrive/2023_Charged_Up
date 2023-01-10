package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class intakeDTF extends CommandBase {
    
    private int frameCount;
    private double duration;

    public intakeDTF(double duration) {
        addRequirements(Robot.Spintake);
        this.duration = duration;
    }

    public void initialize() {
        frameCount = 0;
    }

    public void execute() { 
        Robot.Spintake.spin(2.5,2.5);
        frameCount++;
    }

    public boolean isFinished() {
        return super.isFinished() || frameCount > duration;
    }

    public void end(boolean interrupted) {
        Robot.Spintake.spin(0,0);
    }

}
