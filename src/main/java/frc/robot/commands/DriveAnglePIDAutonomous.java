// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.Drivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveAnglePIDAutonomous extends PIDCommand {

  Drivetrain drivetrain;
  double allowedError = 5;
  double targetAngle;

  /** Creates a new DriveAnglePIDAutonomous. */
  public DriveAnglePIDAutonomous(Drivetrain drivetrain, double angle) {
    super(
        // The controller that the command will use
        new PIDController(0, 0, 0), // These values will need to be tested and tuned.
        // This should return the measurement
        () -> drivetrain.getGyroAngle(),
        // This should return the setpoint (can also be a constant)
        () -> angle,
        // This uses the output
        output -> {
          drivetrain.set(output, -output); // TODO: test to find out which side should be negative
        });
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    addRequirements(drivetrain);
    this.drivetrain = drivetrain;

    targetAngle = angle;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Math.abs(drivetrain.getGyroAngle() - targetAngle) <= allowedError);
  }
}
