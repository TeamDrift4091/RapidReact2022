// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intakeindexshooter;

import edu.wpi.first.math.Pair;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeIndexShooter;

public class Shoot extends CommandBase {
  /** Creates a new Shoot. */
  private IntakeIndexShooter intakeIndexShooter;
  private Timer timer = new Timer();
  private Timer topIndexDelay = new Timer();

  public Shoot(IntakeIndexShooter intakeIndexShooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intakeIndexShooter = intakeIndexShooter;
    addRequirements(intakeIndexShooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    topIndexDelay.reset();
    topIndexDelay.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry ty = limelightTable.getEntry("ty");
    double verticalAngle = ty.getDouble(0);
    double velocity = -1;
    for (Pair<Double, Double> curDistPower : Constants.DISTANCE_TO_POWER) {
      double sampleAngle = curDistPower.getFirst();
      if (sampleAngle < verticalAngle) {
        break;
      }
      velocity = curDistPower.getSecond();
    }
    if (velocity != -1) {
      intakeIndexShooter.setShooterSpeed(velocity);
      if(topIndexDelay.hasElapsed(1)){
        intakeIndexShooter.setTopIndexSpeed(.8);
        intakeIndexShooter.setBottomIndexSpeed(.4);
      }
    } else {
      intakeIndexShooter.setShooterSpeed(1);
      intakeIndexShooter.setTopIndexSpeed(.8);
      intakeIndexShooter.setBottomIndexSpeed(0.6);

    }

    SmartDashboard.putNumber("Shooter Velocity", intakeIndexShooter.getShooterSpeed());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
    intakeIndexShooter.setShooterSpeed(0);
    intakeIndexShooter.setTopIndexSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return timer.hasElapsed(3);
    return false;
  }
}
