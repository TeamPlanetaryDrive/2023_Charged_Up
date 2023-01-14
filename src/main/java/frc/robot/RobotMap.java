/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/* logitech attack 3 garbage xbox superiority gang :muscle: :triumph:
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
*/
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.XboxController;
//import edu.wpi.first.wpilibj.PneumaticsModuleType;
// import edu.wpi.first.wpilibj.buttons.JoystickButton;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
        public static final double PERIODIC_UPDATE_PERIOD = 0.020; // Periodic update period (s)

        // Power Channels
        public static final int DRIVE_POWER_LEFT_FRONT = 837148, DRIVE_POWER_RIGHT_FRONT = 378149,
                        DRIVE_POWER_LEFT_REAR = 893149, DRIVE_POWER_RIGHT_REAR = 81340723, CAMERA_ONE_POWER = 0,
                        CAMERA_TWO_POWER = 1, VRM_POWER = 38578942;

        // Camera Resolution Dimensions
        public static final int CAM_WID = 640, CAM_HEI = 480;

        // piston based
        public static int GRIPPER_CHANNEL_A = 0, GRIPPER_CHANNEL_B = 2;

        // Drive system PID Parameters
        public static final double DRIVE_PID_POSITION_KP = 2.00, // 2.00
                        DRIVE_PID_POSITION_KI = 0.01, // 0.01
                        DRIVE_PID_POSITION_KD = 1.00, // 1.00
                        DRIVE_PID_ANGLE_KP = 0.02, DRIVE_PID_ANGLE_KI = 0.001, DRIVE_PID_ANGLE_KD = 0.0;

        // Digital (0-9, 10-25)
        // 2019 bot
        //public static int LEFT_MOTOR_CHANNEL = 0, RIGHT_MOTOR_CHANNEL = 1;
        // 2022 bot
        public static int LEFT_MOTOR_CHANNEL = 9, RIGHT_MOTOR_CHANNEL = 8;

        // motor based
        //outlining spintake motors, placeholder channels
        //public static int SPINTAKE_CHANNEL_L = 0;
        //public static int SPINTAKE_CHANNEL_R = 6;

        //public static int LEFT_SHOOTER_CHANNEL = 4;

        //public static int RIGHT_SHOOTER_CHANNEL = 5;

        //public static int SHOOTER_AIM_CHANNEL = 1;
        
                        // enconder channels
        // not final yet
        // public static int DRIVETRAIN_ENCODER_CHANNEL_L_A = 0, DRIVETRAIN_ENCODER_CHANNEL_L_B = 1, DRIVETRAIN_ENCODER_CHANNEL_R_A = 2, DRIVETRAIN_ENCODER_CHANNEL_R_B = 3;
        //public static int SHOOTER_ENCODER_CHANNEL_A = 0, SHOOTER_ENCODER_CHANNEL_B = 1;

        // HARDWARE
        
        //public static MotorController lift;

        // state for lift motor
        //public static boolean liftStart = false;

        
        public static JoystickButton aButton, bButton, xButton, yButton, backButton, startButton, leftBumper, rightBumper, leftStickButton, rightStickButton;
        public static final int XBOX_PORT = 1;
        public static final XboxController XController = new XboxController(XBOX_PORT);


        
        public static final int PISTON_CHANNEL_FORWARD = 0, PISTON_CHANNEL_REVERSE = 1;

        // For example to map the left and right motors, you could define the
        // following variables to use with your drivetrain subsystem.
        // public static int leftMotor = 1;
        // public static int rightMotor = 2;

        // If you are using multiple modules, make sure to define both the port
        // number and the module. For example you with a rangefinder:
        // public static int rangefinderPort = 1;
        // public static int rangefinderModule = 1;
        public static void init() {
                //map each button to a JoystickButton
                aButton = new JoystickButton(XController, 1);
                bButton = new JoystickButton(XController, 2);
                xButton = new JoystickButton(XController, 3);
                yButton = new JoystickButton(XController, 4);
                leftBumper = new JoystickButton(XController, 5);
                rightBumper = new JoystickButton(XController, 6);
                backButton = new JoystickButton(XController, 7);
                startButton = new JoystickButton(XController, 8);
                leftStickButton = new JoystickButton(XController, 9);
                rightStickButton = new JoystickButton(XController, 10);
        }
}