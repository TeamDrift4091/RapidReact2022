// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class MotionMagicAuto extends CommandBase {
  private Drivetrain drivetrain;

  private double distance;
  private double allowedError = 4 * Constants.TICKS_PER_REVOLUTION / (Math.PI * 6); // encoder ticks

  /** Creates a new MotionMagicAuto. */
  public MotionMagicAuto(Drivetrain drivetrain, double distance) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
    this.drivetrain = drivetrain;
    this.distance = distance * Constants.TICKS_PER_REVOLUTION / (Math.PI * 6);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetrain.resetEncoderPosition();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.set(ControlMode.MotionMagic, distance, distance);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.arcadeDrive(0, 0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double[] positions = drivetrain.getEncoderPositions();

    return Math.abs(positions[0] - distance) <= allowedError && Math.abs(positions[1] - distance) <= allowedError;
  }
}
