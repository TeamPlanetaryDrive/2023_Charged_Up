package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Lift extends SubsystemBase{
    DoubleSolenoid liftPiston;

    public Lift(){
        liftPiston = new DoubleSolenoid(PneumaticsModuleType.CTREPCM,RobotMap.PISTON_CHANNEL_FORWARD,RobotMap.PISTON_CHANNEL_REVERSE);
        //retract();
    }

    public void extend(){
        liftPiston.set(DoubleSolenoid.Value.kForward);
        System.out.println(liftPiston.isFwdSolenoidDisabled() + "fwd");
    }
    public void retract(){
        liftPiston.set(DoubleSolenoid.Value.kReverse);
        System.out.println(liftPiston.isRevSolenoidDisabled() + "rev");
    }
    public void off(){
        liftPiston.set(DoubleSolenoid.Value.kOff);
    }
    public void toggle(){
        liftPiston.toggle();
    }
}
