// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intakeindexshooter;

import java.lang.reflect.Field;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeIndexShooter;

public class IntakeIndexShooterCommand extends CommandBase {
  private IntakeIndexShooter intakeIndexShooter;
  
  private BooleanSupplier intakeTrigger;
  private BooleanSupplier shootTrigger;

  private Timer intakeDelayTimer;
  private Timer intakeContinueTimer;
  private boolean intakeIsAlreadyActive;
  private boolean intakeIsAlreadyOff;

  private Timer ejectContinueTimer;

  /** Creates a new IntakeIndexShooterCommand. */
  public IntakeIndexShooterCommand(IntakeIndexShooter intakeIndexShooter, BooleanSupplier intakeTrigger, BooleanSupplier shootTrigger) {
    addRequirements(intakeIndexShooter);
    this.intakeIndexShooter = intakeIndexShooter;

    this.intakeTrigger = intakeTrigger;
    this.shootTrigger = shootTrigger;

    this.intakeDelayTimer = new Timer();
    this.intakeContinueTimer = new Timer();
    this.intakeIsAlreadyActive = false;
    this.intakeIsAlreadyOff = true;

    this.ejectContinueTimer = new Timer();

    // Reflection - Start intakeContinueTimer at 1 second elapsed
    try {
      Class timerClass = Timer.class;
      Field t_startTime = timerClass.getDeclaredField("m_startTime");
      t_startTime.setAccessible(true);
      double intakeContinueTimer_startTime = (double) t_startTime.get(intakeContinueTimer);
      intakeContinueTimer_startTime -= 1000;
      t_startTime.set(intakeContinueTimer_startTime, intakeContinueTimer_startTime);
    } catch (NoSuchFieldException | IllegalAccessException exception) {
      exception.printStackTrace();
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ejectContinueTimer.start();
  }

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
      intakeIndexShooter.setShooterSpeed(1);
      intakeIndexShooter.setTopIndexSpeed(.6);
    // Eject wrong color
    } else if (!intakeIndexShooter.isCorrectColor()) {
      intakeIndexShooter.setShooterSpeed(.6);
      intakeIndexShooter.setTopIndexSpeed(.6);
      ejectContinueTimer.reset();
      ejectContinueTimer.start();
    // Do nothing
    } else if(ejectContinueTimer.hasElapsed(1)) {
      ejectContinueTimer.stop();
      intakeIndexShooter.setShooterSpeed(0);
      intakeIndexShooter.setTopIndexSpeed(0);
    }

    // Intake
    if (shouldIntake && !(bottomSlotContainsBall && topSlotContainsBall)) {
      // initialize
      if (!intakeIsAlreadyActive) {
        intakeIndexShooter.extendIntakeArm();
        intakeDelayTimer.start();
        intakeContinueTimer.reset();
        intakeContinueTimer.stop();
        intakeIsAlreadyActive = true;
        intakeIsAlreadyOff = false;
      }
      // execute
      if(intakeDelayTimer.hasElapsed(0.5)){
        intakeIndexShooter.setIntakeSpeed(-0.5); // Motor must be reversed

        // Only runs until the bottomSlot is full because of the outermost if statement
        intakeIndexShooter.setBottomIndexSpeed(.6);
      } else {
        intakeContinueTimer.stop();
      }
    // end
    } else {
      if (!intakeIsAlreadyOff) {
        intakeIndexShooter.retractIntakeArm();
        intakeDelayTimer.reset();
        intakeDelayTimer.stop();
        intakeContinueTimer.start();
        intakeIsAlreadyActive = false;
        intakeIsAlreadyOff = true;
      }

      intakeIndexShooter.setIntakeSpeed(0);

      // Advance Ball and keep motor spinning after intake retracts
      if (!intakeContinueTimer.hasElapsed(1) || (bottomSlotContainsBall && !topSlotContainsBall)) {
        intakeIndexShooter.setBottomIndexSpeed(.6);
      } else {
        intakeContinueTimer.stop();
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
