package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class scuffedCommand extends CommandBase {

    int frameCount = 0;
    
    public scuffedCommand() {
        addRequirements(Robot.Drive, Robot.Spintake);
    }

    public void initialize() {
        frameCount = 0;
    }

    public void execute() {
        if(frameCount < 50) {

        } else if(frameCount < 60) {
            Robot.Drive.drive(-.75, -.75);
        } else if(frameCount < 62) {
            Robot.Spintake.spin(-.7,-.7);
        } else if(frameCount < 85) {
            Robot.Spintake.spin(1, 1);
        } else {
            Robot.Drive.drive(-.75, -.75);
        }
        frameCount++;
    }

    public boolean isFinished() {
        return super.isFinished() || frameCount > 190;
    }

    public void end(boolean interrupted) {
        Robot.Spintake.spin(0, 0);
        Robot.Drive.drive(0, 0);
    }

}
