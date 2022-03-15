// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intakeindexshooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeIndexShooter;

public class Intake extends CommandBase {
  /** Creates a new Intake. */
  IntakeIndexShooter intakeIndexShooter;
  Timer timerCommand = new Timer();
  Timer intakeContinueTimer = new Timer();
  Timer intakeDelayTimer = new Timer();
  
  public Intake(IntakeIndexShooter intakeIndexShooter) {
    // Use addReq uirements() here to declare subsystem dependencies.
    this.intakeIndexShooter = intakeIndexShooter;
    addRequirements(intakeIndexShooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intakeIndexShooter.extendIntakeArm();
    intakeDelayTimer.start();
    intakeContinueTimer.reset();
    timerCommand.reset();
    timerCommand.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intakeIndexShooter.setIntakeSpeed(-0.5); // Motor must be reversed
    intakeIndexShooter.setBottomIndexSpeed(.6);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeIndexShooter.setIntakeSpeed(0); // Motor must be reversed
    intakeIndexShooter.setBottomIndexSpeed(0);
    intakeIndexShooter.retractIntakeArm();
        intakeDelayTimer.reset();
        intakeDelayTimer.stop();
        intakeContinueTimer.start();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timerCommand.hasElapsed(5);
  }
}
