// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class IntakeCommand extends CommandBase {
  /** Creates a new IntakeCommand. */

  Intake intake;
  Timer timer;

  boolean isAlreadyActive;
  DoubleSupplier forwardThrottle;
  DoubleSupplier reverseThrottle;

  public IntakeCommand(Intake intake, DoubleSupplier forwardThrottle, DoubleSupplier reverseThrottle) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = intake;
    addRequirements(intake);
    timer = new Timer();

    this.isAlreadyActive = false;
    this.forwardThrottle = forwardThrottle;
    this.reverseThrottle = reverseThrottle;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double forward = forwardThrottle.getAsDouble();
    double reverse = reverseThrottle.getAsDouble();
    double activeThrottle = forward >= reverse ? forward : reverse;
    int activeDirection = forward >= reverse ? -1 : 1;
    
    if (activeThrottle > .05) {
      // initialize
      if (!isAlreadyActive) {
        intake.lowerIntakeArm();
        timer.start();
        isAlreadyActive = true;
      }
      // execute
      if(timer.hasElapsed(0.5)){
        intake.setIntakeSpeed(0.75 * activeDirection);
      }
    // end
    } else {
      intake.raiseIntakeArm();
      intake.setIntakeSpeed(0);
      timer.reset();
      timer.stop();
  
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
