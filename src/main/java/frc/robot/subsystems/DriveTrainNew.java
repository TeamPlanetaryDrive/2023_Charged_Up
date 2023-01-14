package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.Victor;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import frc.robot.RobotMap;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveTrainNew extends SubsystemBase {
    DifferentialDrive robotDrive;
    Victor lMotor, rMotor;
    public Encoder encoderL, encoderR;
    
    public DriveTrainNew() {
        super();
        lMotor = new Victor(RobotMap.LEFT_MOTOR_CHANNEL);
        rMotor = new Victor(RobotMap.RIGHT_MOTOR_CHANNEL);
        rMotor.setInverted(true);
        robotDrive = new DifferentialDrive(lMotor, rMotor);
        robotDrive.setSafetyEnabled(false);
        encoderL = new Encoder(RobotMap.DRIVETRAIN_ENCODER_CHANNEL_L_A, RobotMap.DRIVETRAIN_ENCODER_CHANNEL_L_B);
        encoderR = new Encoder(RobotMap.DRIVETRAIN_ENCODER_CHANNEL_R_A, RobotMap.DRIVETRAIN_ENCODER_CHANNEL_R_B, true);
      }
}