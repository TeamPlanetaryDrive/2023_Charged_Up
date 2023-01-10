package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class auto1 extends CommandBase {
    
    int frameCount = 0;
    int moveDuration = 70;

    public auto1() {
        addRequirements(Robot.Drive);
    }

    public void initialize() {
        frameCount = 0;
    }

    public void execute() {
        System.out.println("edcndo");
        Robot.Drive.drive(-.7, -.7);
        frameCount++;
    }

    public boolean isFinished() {
        return false || frameCount > moveDuration;
    }

    public void end(boolean interrupted) {
    }

}
