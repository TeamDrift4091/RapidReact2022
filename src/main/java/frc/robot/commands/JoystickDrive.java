// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
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
  public void initialize() {
    drivetrain.resetEncoderPosition();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double updatedY = joyY.getAsDouble();
    double updatedX = joyX.getAsDouble();
    // drivetrain.arcadeDrive(updatedY, updatedX, true);
    System.out.println("JoyY: " + updatedY);
    drivetrain.set(ControlMode.MotionMagic, updatedY* Constants.TICKS_PER_REVOLUTION*2, updatedY* Constants.TICKS_PER_REVOLUTION*2);
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
