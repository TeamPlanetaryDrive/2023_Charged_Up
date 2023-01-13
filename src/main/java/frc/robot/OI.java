/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.*;
import frc.robot.commands.vision.*;


/**
 * This class is what binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {

    private static final int LEFT_STICK_PRESS = 0, RIGHT_STICK_PRESS = 1;    

  public OI() {
    // TODO: remap the buttons to things

    // RobotMap.aButton.whenPressed(new turnToGoal());    
    // RobotMap.bButton.whenPressed(new testCommand());
    // RobotMap.xButton.whenPressed(new dropShooter());
    // RobotMap.yButton.whenPressed(new climb()); 
    // RobotMap.startButton.whenPressed(new setLift(true));
    // RobotMap.backButton.whenPressed(new setLift(false));
    // RobotMap.leftBumper.whenPressed(new SequentialCommandGroup(new intakeCBT(2), new intakeDTF(30)));
    // RobotMap.rightBumper.whenPressed(new intakeDTFSlow(25));
    
    // RobotMap.leftStickButton.whenHeld(new movement(LEFT_STICK_PRESS));
    // RobotMap.rightStickButton.whenHeld(new movement(RIGHT_STICK_PRESS));
    
    /*
    Goals for Xbox Controller Button Pressing Mapping
    -------------------------------------------------
    D-Pad: Fine Movement Control (X)
    Left Stick: Movement (X)
    Right Stick: Manual Shooter Control (X)
    Left Trigger: Spintake Wheels (H)
    Right Trigger: Spin Shooter Wheels (H)
    Left Bumper: Group Command Intake (P)
    Right Bumper: Group Command Shoot (P)
    X: Turn to Ball (P)
    Y: Climb (P)
    A: Turn to Goal (P)
    B: ? [Reset?, Debug?] (P)
    */
    
    
  }
}
