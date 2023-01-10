package frc.robot.commands.vision;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class turnToBall extends CommandBase {

    NetworkTableEntry contourInfo;
    boolean finish;
    double[] position;
    
    public turnToBall() {
        addRequirements(Robot.Cameras, Robot.Drive);
        contourInfo = NetworkTableInstance.getDefault().getTable("datatable").getEntry("balls");
    }

    public void initialize() {
        finish = false;
        System.out.println("mmm balls");
        Robot.Cameras.doBall = true;
    }

    public void execute() {
        if(Robot.Cameras.countBall < 10) {
            return;
        }
        position = contourInfo.getDoubleArray(new double[] {-1, -1});
    }

    public boolean isFinished() {
        return true;
    }

    public void end(boolean interrupted) {
        Robot.Cameras.doBall = false;
    }

}
