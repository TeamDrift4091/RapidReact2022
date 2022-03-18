// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intakeindexshooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeIndexShooter;

public class Shoot extends CommandBase {
  /** Creates a new Shoot. */
  private IntakeIndexShooter intakeIndexShooter;
  private Timer timer = new Timer();

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
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      intakeIndexShooter.setShooterSpeed(1);
      intakeIndexShooter.setTopIndexSpeed(.6);
      intakeIndexShooter.setBottomIndexSpeed(.4);;
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
    return timer.hasElapsed(3);
  }
}
