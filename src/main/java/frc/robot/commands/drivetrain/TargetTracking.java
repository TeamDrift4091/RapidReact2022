// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class TargetTracking extends CommandBase {

  private Drivetrain drivetrain;
  //private Timer timer = new Timer();
  private double horizontalAngle;
  private double updatedHorizontalAngle = 1;
  private double threshold = 1;
  

  /** Creates a new BallTracking. */
  public TargetTracking(Drivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetrain.arcadeDrive(0, 0, false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = limelightTable.getEntry("tx");

    horizontalAngle = tx.getDouble(0);
    double steeringAdjustment;

    if(horizontalAngle == 0){
      drivetrain.arcadeDrive(0, .15, false);
    } else if(horizontalAngle < threshold ){
        updatedHorizontalAngle = horizontalAngle;
    }
    
    if(Math.abs(horizontalAngle) > threshold){ 
      steeringAdjustment = 0.06 * Math.sqrt(Math.abs(horizontalAngle)) * Math.signum(horizontalAngle);
      drivetrain.arcadeDrive(0, steeringAdjustment, false);
    }
   
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.arcadeDrive(0, 0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return timer.hasElapsed(4);
    return updatedHorizontalAngle < threshold;
  }
}
