// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class TargetTrackingDistance extends CommandBase {
  private Drivetrain drivetrain;
  
  /** Creates a new TargetTrackingDistance. */
  public TargetTrackingDistance(Drivetrain drivetrain) {
    addRequirements(drivetrain);
    this.drivetrain = drivetrain;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.set(ControlMode.PercentOutput, .2, .2);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.set(ControlMode.PercentOutput, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Second to last element in list is the furthest it can be.
    NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry ty = limelightTable.getEntry("ty");
    double verticalAngle = ty.getDouble(0);
    return verticalAngle < Constants.DISTANCE_TO_POWER.get(Constants.DISTANCE_TO_POWER.size()-2).getFirst() && verticalAngle != 0;
  }
}
