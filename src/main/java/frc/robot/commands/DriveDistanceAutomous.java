// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class DriveDistanceAutomous extends CommandBase {

  private Drivetrain drivetrain;
  private double targetDistance;

  private double allowedError = 100;

  /** Creates a new DriveDistance. */
  public DriveDistanceAutomous(Drivetrain drivetrain, double targetDistance) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
    this.drivetrain = drivetrain;
    this.targetDistance = targetDistance;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetrain.resetEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.set(ControlMode.MotionMagic, targetDistance, targetDistance);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.set(0,0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double[] positions = drivetrain.getPositions();

    return Math.abs(positions[0] - targetDistance) <= allowedError && Math.abs(positions[1] - targetDistance) <= allowedError;
  }
}
