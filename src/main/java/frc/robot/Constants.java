// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Arrays;
import java.util.List;

import edu.wpi.first.math.Pair;
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

    public static final int TIMEOUT_MS = 30;

    // Target Tracking Constants
    public static double TARGET_TRACKING_P = 0.03;
    public static double TARGET_TRACKING_I = 0.04;

    // Drivetrain Constants
    public static int FRONT_LEFT_PORT = 1;
    public static int BACK_LEFT_PORT = 2;
    public static int FRONT_RIGHT_PORT = 3;
    public static int BACK_RIGHT_PORT = 4;

    public static double WHEEL_BASE_WIDTH_INCHES = 21.875; // inches
    public static double WHEEL_BASE_WIDTH = Units.inchesToMeters(WHEEL_BASE_WIDTH_INCHES); // meters

    public static int TICKS_PER_REVOLUTION = 4096;

        // Encoder ticks per rotation of motor * gear ratio / diameter of wheel (ft)
    public static double ENCODER_TICKS_PER_FOOT = TICKS_PER_REVOLUTION * 7.6 / (Math.PI * .5); // feet
    public static double ENCODER_TICKS_PER_METER = Units.feetToMeters(ENCODER_TICKS_PER_FOOT); // meters

        // Rotations per second * gear ratio / diameter of wheel
    public static double MAX_SPEED_FEET_PER_SECOND = ((6380/60.)/7.6)/(.5*Math.PI); // feet per second;
    public static double MAX_SPEED = Units.feetToMeters(MAX_SPEED_FEET_PER_SECOND); // meters per second
        // Tangential velocity / radius
    public static double MAX_ANGULAR_VELOCITY = MAX_SPEED/(WHEEL_BASE_WIDTH/2.); // radians per second;

    // Climber Constants
    public static int WINCH_MOTOR = 9;

    // Index and Shooter Constants
    public static int TOP_INDEX_MOTOR = 7;
    public static int BOTTOM_INDEX_MOTOR = 8;
    public static int SHOOTER_MOTOR = 6;
    public static int INTAKE_MOTOR = 5;

    public static List<Pair<Double, Double>> DISTANCE_TO_POWER = Arrays.asList(
        // Go from greatest to least
        new Pair<>(13.30, 1.),
        new Pair<>(10.50, 1.0),
        new Pair<>(8.5, 1.0),
        new Pair<>(0.0, -1.0)
      );

    // Pneumatic Constants
    public static int INTAKE_SOLENOID_DOWN = 6;
    public static int INTAKE_SOLENOID_UP = 1;

}


