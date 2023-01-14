package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class TestCommand extends CommandBase {

    public TestCommand() {

    }

    public void initialize() {

    }

    public void execute() {
        System.out.println("Test command executed");
    }

    public boolean isFinished() {
        return true;
    }

    public void end(boolean interrupted) {

    }
    
}
