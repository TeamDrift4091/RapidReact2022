// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexShooter;

public class IndexShooterCommand extends CommandBase {
  /** Creates a new IndexShooterCommand. */
  private IndexShooter indexShooter;
  private BooleanSupplier isTriggerPressed;

  public IndexShooterCommand(IndexShooter indexShooter, BooleanSupplier isTriggerPressed) {
    this.indexShooter = indexShooter;
    this.isTriggerPressed = isTriggerPressed;
    addRequirements(indexShooter);
  }

  /**
   * Shoots a ball in the top slot.
   * @param speed value between -1 and 1 representing speed at which motors will spin.
   */
  public void shootBall(double speed){
    indexShooter.setTopIndexSpeed(speed);
    indexShooter.setMiddleIndexSpeed(speed);
  }

  /**
   * Moves a ball from the lower slot to the upper slot.
   * @param speed value between -1 and 1 representing speed at which motors will spin.
   */
  public void advanceBall(double speed) {
    indexShooter.setBottomIndexSpeed(speed);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(isTriggerPressed.getAsBoolean()){
      shootBall(0.7);
    } else if (indexShooter.isCorrectColor()){
      shootBall(0.3);
    } else {
      shootBall(0);
    }

    if(indexShooter.isUpperSlotEmpty()){
      advanceBall(.5);
    } else {
      advanceBall(0);
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
