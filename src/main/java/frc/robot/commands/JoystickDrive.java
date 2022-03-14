// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class JoystickDrive extends CommandBase {

  private Drivetrain drivetrain;

  private DoubleSupplier joyY;
  private DoubleSupplier joyX;

  /** Creates a new JoystickDrive. */
  public JoystickDrive(Drivetrain drivetrain, DoubleSupplier joyY, DoubleSupplier joyX) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
    this.drivetrain = drivetrain;
    this.joyY = joyY;
    this.joyX = joyX;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double updatedY = joyY.getAsDouble();
    double updatedX = joyX.getAsDouble();
    // Options:
    // drivetrain.arcadeDrive(updatedY, updatedX, true); // Default
    drivetrain.arcadeDrive(updatedY, updatedX*updatedX, true); // Stronger ramping on rotation
    // drivetrain.arcadeDrive(updatedY, updatedX*.5, true); // Limited max rotation
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.arcadeDrive(0, 0, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
