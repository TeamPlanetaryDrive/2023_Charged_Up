package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Victor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Intake extends SubsystemBase {

    Victor lMotor, rMotor;

    public Intake() {
        lMotor = new Victor(RobotMap.SPINTAKE_CHANNEL_L);
        rMotor = new Victor(RobotMap.SPINTAKE_CHANNEL_R);
    }
    
    public void periodic() {
        double[] speeds = getSpeed();
        spin(speeds[0]*0.7,speeds[1]*0.7);
    }

    public static double[] getSpeed() {
        if(RobotMap.XController.getRightTriggerAxis()>0)
            return(new double[] {RobotMap.XController.getRightTriggerAxis(), RobotMap.XController.getRightTriggerAxis()});
        else
            return(new double[] {-RobotMap.XController.getLeftTriggerAxis(), -RobotMap.XController.getLeftTriggerAxis()});
    }

    public void spin(double left, double right){
        lMotor.set(left);
        rMotor.set(right);
    }
    
}
