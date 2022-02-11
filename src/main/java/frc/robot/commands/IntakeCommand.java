// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class IntakeCommand extends CommandBase {
  /** Creates a new IntakeCommand. */

  Intake intake;
  Timer timer;

  BooleanSupplier activate;
  boolean isAlreadyActive;

  public IntakeCommand(Intake intake, BooleanSupplier activate) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = intake;
    addRequirements(intake);
    timer = new Timer();

    this.activate = activate;
    this.isAlreadyActive = false;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (activate.getAsBoolean()) {
      // initialize
      if (!isAlreadyActive) {
        intake.lowerIntakeArm();
        timer.start();
      }
      // execute
      if(timer.hasElapsed(1)){
        intake.setIntakeSpeed(0.5);
      }
    // end
    } else {
      intake.raiseIntakeArm();
      intake.setIntakeSpeed(0);
  
      isAlreadyActive = false;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
