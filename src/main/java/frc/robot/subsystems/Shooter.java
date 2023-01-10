package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Shooter extends SubsystemBase{

    private Talon motorC;

    public Encoder shooterEnc;

    public Shooter() {
        motorC = new Talon(RobotMap.SHOOTER_AIM_CHANNEL);
        shooterEnc = new Encoder(RobotMap.SHOOTER_ENCODER_CHANNEL_A, RobotMap.SHOOTER_ENCODER_CHANNEL_B);
        shooterEnc.setDistancePerPulse(((8/3)*Math.PI)/256.); //[n/q] sets to distance of n every q pulses
        shooterEnc.reset();
    }

    public void rotate(double central) {
        motorC.set(1.1*central);
    }

    public double getRotateSpeeds(){
        double yaxis = RobotMap.XController.getRightY();
        return Math.abs(yaxis)>.2?yaxis*.65:0;
    }

    public double[] getSpinSpeeds(){
        double xaxis = RobotMap.XController.getRightX();
        xaxis = Math.abs(xaxis)>.2?xaxis:0;
        return new double[] {xaxis,xaxis};
    }

    public void periodic(){
        double rott = getRotateSpeeds();
        rotate(rott);
    }
}
