package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class shoot extends CommandBase {
    
    double currentSpeed;
    double v;
    final double ANG = Math.toRadians(55);
    final double GOAL_HEI = 8.666667, ROBO_HEI = 4;
    final double H = GOAL_HEI - ROBO_HEI;
    final double G = 32.17;
    double d = 15;

    public shoot() {
        addRequirements(Robot.Gun, Robot.Spintake);
    }

    public void initialize() {
        
    }

    public void execute() {
        v = Math.sqrt((-G*d*d)/(2*Math.cos(ANG)*Math.cos(ANG)*(H-d*Math.tan(ANG))));
        currentSpeed = Robot.Gun.shooterEnc.getRate();
        if(currentSpeed < v) {
            Robot.Spintake.spin(1.4, 1.4);
        }
    }

    public boolean isFinished() {
        return super.isFinished();
    }

    public void end(boolean interrupted) {

    }

}
