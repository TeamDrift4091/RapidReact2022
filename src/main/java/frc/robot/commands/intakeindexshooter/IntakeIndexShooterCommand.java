// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intakeindexshooter;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.Pair;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeIndexShooter;

public class IntakeIndexShooterCommand extends CommandBase {
  private IntakeIndexShooter intakeIndexShooter;
  
  private BooleanSupplier intakeTrigger;
  private BooleanSupplier shootTrigger;

  private Timer intakeDelayTimer;
  private Timer intakeContinueTimer;
  private Timer ejectContinueTimer;
  private boolean intakeIsAlreadyActive;
  private boolean intakeIsAlreadyOff;

  /** Creates a new IntakeIndexShooterCommand. */
  public IntakeIndexShooterCommand(IntakeIndexShooter intakeIndexShooter, BooleanSupplier intakeTrigger, BooleanSupplier shootTrigger) {
    addRequirements(intakeIndexShooter);
    this.intakeIndexShooter = intakeIndexShooter;

    this.intakeTrigger = intakeTrigger;
    this.shootTrigger = shootTrigger;

    this.intakeDelayTimer = new Timer();
    this.intakeContinueTimer = new Timer();
    this.ejectContinueTimer = new Timer();
    this.intakeIsAlreadyActive = false;
    this.intakeIsAlreadyOff = false;
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
      // TODO: Should the robot shoot if the limelight can't see the target?
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
        intakeIndexShooter.setTopIndexSpeed(.6);
      }
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
