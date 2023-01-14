package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class ArcadeMovement extends CommandBase {

    // set to 0 if not constant thrust - you want it to go faster when u push joystick further
    // otherwise, it will just use the constant as the thrust for everything.
    public double thrustConstant = 0;

    public XboxController driveStick;

    public ArcadeMovement() {
        driveStick = RobotMap.XController;

    }

    public void initialize() {

    }
    
    @Override
    public void execute() {
        if (thrustConstant != 0) {

            if(driveStick.getRightY() >=0) {
                
            }


            Robot.Drive.arcadeDrive(thrustConstant, thrustConstant);
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
