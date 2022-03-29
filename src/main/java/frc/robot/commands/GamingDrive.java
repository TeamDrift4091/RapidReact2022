// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class GamingDrive extends CommandBase {

  private Drivetrain drivetrain;
  private DoubleSupplier gas, brake, steering;
  private BooleanSupplier eBrake;

  private boolean quickTurn;

  private double velocity;
  private double velocityDecay = .95;

  private long startTime;

  /**
   * Driving style only intended for messing around, not for competition.  Use with care.
   * @param drivetrain
   * @param gas throttle/trigger controlling acceleration [0,1]
   * @param brake throttle/trigger controlling deceleration [0,1]
   * @param steering joystick steering [-1, 1]
   * @param eBrake button that stops all motion
   */
  public GamingDrive(Drivetrain drivetrain, DoubleSupplier gas, DoubleSupplier brake, DoubleSupplier steering, BooleanSupplier eBrake) {
    addRequirements(drivetrain);
    this.drivetrain = drivetrain;
    this.gas = gas;
    this.brake = brake;
    this.steering = steering;
    this.eBrake = eBrake;
    this.velocity = 0;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = System.nanoTime();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double acceleration;
    double gasMagnitude = gas.getAsDouble();
    double brakeMagnitude = brake.getAsDouble();
    if (brakeMagnitude > .1) {
      acceleration = -brakeMagnitude;
    } else {
      acceleration = gasMagnitude;
    }

    // Calculating time since last call to execute
    // Necessary for calculating the change in the velocity
    long currentTime = System.nanoTime();
    long timeDelta = currentTime - startTime;
    startTime = currentTime;
    double timeDeltaInSeconds = timeDelta / 1000000000.;

    // Adding the accleration to the velocity
    if (Math.abs(acceleration) > Math.abs(velocity) || Math.signum(acceleration) != Math.signum(velocity)) {
      velocity += acceleration * timeDeltaInSeconds;
    }

    // Limit velocity
    double max = 1;

    velocity = Math.abs(velocity) > max ? max * Math.signum(velocity) : velocity;

    if (eBrake.getAsBoolean()) {
      velocity = 0;
    }

    if (Math.abs(acceleration) == 0) {
      velocity *= velocityDecay;
    }

    // Calculate quickTurn
    quickTurn = Math.abs(velocity) < .1;

    // Fix reversing weirdness
    double steeringDouble = steering.getAsDouble();
    steeringDouble *= Math.signum(velocity);

    // drivetrain.arcadeDrive(velocity, steering.getAsDouble(), false);
    drivetrain.curvatureDrive(velocity, steeringDouble, quickTurn);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    velocity = 0;
    drivetrain.arcadeDrive(0, 0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
