// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    // Drivetrain Constants
    public static int FRONT_LEFT_PORT = 1;
    public static int MIDDLE_LEFT_PORT = 2;
    public static int BACK_LEFT_PORT = 3;
    public static int FRONT_RIGHT_PORT = 4;
    public static int MIDDLE_RIGHT_PORT = 5;
    public static int BACK_RIGHT_PORT = 6;
    
    //Intake Constants
    public static int INTAKE_PORT = 7;

    public static int INTAKE_DOWN_RIGHT = 0;
    public static int INTAKE_UP_RIGHT = 4;

    public static int INTAKE_DOWN_LEFT = 1;
    public static int INTAKE_UP_LEFT = 5;

    //Climber Constants
    public static int WINCH_MOTOR = 8;

    //xBox Constants
    public static int MOVE_WINCH_BUTTON = 4; 
}


