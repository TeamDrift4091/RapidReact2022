// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeIndexShooter;

public class IntakeIndexShooterCommand extends CommandBase {
  private IntakeIndexShooter intakeIndexShooter;
  
  private BooleanSupplier intakeTrigger;
  private BooleanSupplier shootTrigger;

  private Timer intakeDelayTimer;
  private boolean intakeIsAlreadyActive;

  /** Creates a new IntakeIndexShooterCommand. */
  public IntakeIndexShooterCommand(IntakeIndexShooter intakeIndexShooter, BooleanSupplier intakeTrigger, BooleanSupplier shootTrigger) {
    addRequirements(intakeIndexShooter);
    this.intakeIndexShooter = intakeIndexShooter;

    this.intakeTrigger = intakeTrigger;
    this.shootTrigger = shootTrigger;

    this.intakeDelayTimer = new Timer();
    this.intakeIsAlreadyActive = false;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean shouldIntake = intakeTrigger.getAsBoolean();
    boolean shouldShoot = shootTrigger.getAsBoolean();
    boolean topSlotContainsBall = intakeIndexShooter.getUpperSlot();
    boolean bottomSlotContainsBall = intakeIndexShooter.getLowerSlot();

    // Shoot
    if (shouldShoot) {
      // TODO: Dynamic distance adjustment
      intakeIndexShooter.setShooterSpeed(.8);
      intakeIndexShooter.setTopIndexSpeed(.6);
    // Eject wrong color
    } else if (!intakeIndexShooter.isCorrectColor()) {
      intakeIndexShooter.setShooterSpeed(.6);
      intakeIndexShooter.setTopIndexSpeed(.6);
    // Do nothing
    } else {
      intakeIndexShooter.setShooterSpeed(0);
      intakeIndexShooter.setTopIndexSpeed(0);
    }

    // Intake
    if (shouldIntake && !(bottomSlotContainsBall && topSlotContainsBall)) {
      // initialize
      if (!intakeIsAlreadyActive) {
        intakeIndexShooter.extendIntakeArm();
        intakeDelayTimer.start();
        intakeIsAlreadyActive = true;
      }
      // execute
      if(intakeDelayTimer.hasElapsed(0.5)){
        intakeIndexShooter.setIntakeSpeed(-0.5); // Motor must be reversed

        // Only runs until the bottomSlot is full because of the outermost if statement
        intakeIndexShooter.setBottomIndexSpeed(.5);
      }
    // end
    } else {
      intakeIndexShooter.retractIntakeArm();
      intakeIndexShooter.setIntakeSpeed(0);
      intakeDelayTimer.reset();
      intakeDelayTimer.stop();
  
      intakeIsAlreadyActive = false;

      // Advance lower ball
      if (bottomSlotContainsBall && !topSlotContainsBall) {
        intakeIndexShooter.setBottomIndexSpeed(.5);
      } else {
        intakeIndexShooter.setBottomIndexSpeed(0);
      }
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
