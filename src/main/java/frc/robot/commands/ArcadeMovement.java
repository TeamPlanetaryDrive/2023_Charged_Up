package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class ArcadeMovement extends CommandBase {

    // set to 0 if not constant thrust - you want it to go faster when u push joystick further
    // otherwise, it will just use the constant as the thrust for everything.

    // {thrust const, thrustX, thrustY}
    public double[] thrustConstant = {0, 0, 0};
    public XboxController controller;

    public ArcadeMovement() {
        controller = RobotMap.XController;
    }

    public void initialize() {

    }
    
    @Override
    public void execute() {
        
        if (thrustConstant[0] >= 0) {
            thrustConstant[1] = thrustConstant[0];
            thrustConstant[2] = thrustConstant[0];
        } else if (thrustConstant[0] == 0) {
            thrustConstant[1] = controller.getRightX();
            thrustConstant[2] = controller.getRightY(); 
        }

        if(controller.getRightY() >= 0) {
            if(controller.getRightX() >= 0) {
                Robot.Drive.arcadeDrive(thrustConstant[1], -thrustConstant[2]);
            } else {
                Robot.Drive.arcadeDrive(-thrustConstant[1], -thrustConstant[2]);
            }
        } else {
            if(controller.getRightX() >= 0) {
                Robot.Drive.arcadeDrive(thrustConstant[1], thrustConstant[2]);
            } else {
                Robot.Drive.arcadeDrive(-thrustConstant[1], thrustConstant[2]);
            }
        }
    }

    @Override
    public boolean isFinished() {
        return super.isFinished();
    }

    public void end() {
        Robot.Drive.arcadeDrive(0, 0);
    }

    
}
