// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class TargetTrackingClockwiseBias extends CommandBase {
  private Drivetrain drivetrain;
  private PIDController controller;
  private double horizontalAngle;
  private double threshold = 1;
  private int inThreshold;
  /** Creates a new BallTracking. */
  public TargetTrackingClockwiseBias(Drivetrain drivetrain) {
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);
    controller = new PIDController(Constants.TARGET_TRACKING_P, Constants.TARGET_TRACKING_I, 0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetrain.arcadeDrive(0, 0, false);
    inThreshold = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = limelightTable.getEntry("tx");
    horizontalAngle = tx.getDouble(0);

    if(horizontalAngle == 0){
      drivetrain.arcadeDrive(0, .15, false);
    } else {
      drivetrain.arcadeDrive(0, controller.calculate(0, Math.sqrt(Math.abs(horizontalAngle)) * Math.signum(horizontalAngle)), false);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.arcadeDrive(0, 0, false);
    controller.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {    
    if (Math.abs(horizontalAngle) < threshold && horizontalAngle != 0) {
      inThreshold++;
    } else {
      threshold = 0;
    }
    return inThreshold > 50;
  }
}
