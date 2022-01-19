// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class DriveAngleAutonomous extends CommandBase {

  Drivetrain drivetrain;
  double targetAngle;

  double allowedError = 5;

  /** Creates a new DriveAngleAutonomous. */
  public DriveAngleAutonomous(Drivetrain drivetrain, double angle) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
    this.drivetrain = drivetrain;
    this.targetAngle = angle;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetrain.resetGyro();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.set(ControlMode., leftMagnitude, rightMagnitude);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
