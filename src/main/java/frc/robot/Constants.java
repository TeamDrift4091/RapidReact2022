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
    public static int BACK_LEFT_PORT = 2;
    public static int FRONT_RIGHT_PORT = 3;
    public static int BACK_RIGHT_PORT = 4;

    // Climber Constants
    public static int WINCH_MOTOR = 7;

    // Index and Shooter Constants
    public static int TOP_INDEX_MOTOR = 8;
    public static int BOTTOM_INDEX_MOTOR = 9;
    public static int SHOOTER_MOTOR = 10;
    public static int INTAKE_MOTOR = 5;

    // Pneumatic Constants
    public static int INTAKE_SOLENOID_DOWN = 7;
    public static int INTAKE_SOLENOID_UP = 0;
}


