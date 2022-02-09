// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

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

    public static double WHEEL_BASE_WIDTH_INCHES = 21.875; // inches
    public static double WHEEL_BASE_WIDTH = Units.inchesToMeters(WHEEL_BASE_WIDTH_INCHES); // meters

        // Encoder ticks per rotation of motor * gear ratio / diameter of wheel (ft)
    public static double ENCODER_TICKS_PER_FOOT = 4096 * 7.6 / (Math.PI * .5); // feet
    public static double ENCODER_TICKS_PER_METER = Units.feetToMeters(ENCODER_TICKS_PER_FOOT); // meters

        // Rotations per second * gear ratio / diameter of wheel
    public static double MAX_SPEED_FEET_PER_SECOND = ((6380/60.)/7.6)/(.5*Math.PI); // feet per second;
    public static double MAX_SPEED = Units.feetToMeters(MAX_SPEED_FEET_PER_SECOND); // meters per second
        // Tangential velocity / radius
    public static double MAX_ANGULAR_VELOCITY = MAX_SPEED/(WHEEL_BASE_WIDTH/2.); // radians per second;
}
