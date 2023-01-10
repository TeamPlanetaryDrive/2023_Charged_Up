package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class setLift extends CommandBase{

    boolean mode;

    public setLift(boolean mode) {
        addRequirements(Robot.Climb);
        this.mode = mode;
    }

    public void initialize() {
        if(mode){
            Robot.Climb.extend();
        }else{
            Robot.Climb.retract();
        }
    }

    public void execute() {
         
    }

    public boolean isFinished() {
        return super.isFinished();
    }

    public void end(boolean interrupted) {
        
    }

}