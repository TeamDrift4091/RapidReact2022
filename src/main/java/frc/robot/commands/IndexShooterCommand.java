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

  public boolean isUpperSlotEmpty(){
    return indexShooter.getRangeInches() > 2 && indexShooter.getColorMatch() == null;
  }

  public void shootBall(double speed){
    indexShooter.setTopIndexSpeed(speed);
    indexShooter.setMiddleIndexSpeed(speed);
  }

  public void advanceBall(){
    indexShooter.setBottomIndexSpeed(0.5);
  }

  public boolean isWrongColor(){
    return indexShooter.getColorMatch() != indexShooter.teamColor && indexShooter.getColorMatch() != null;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(isTriggerPressed.getAsBoolean()){
      shootBall(0.7);
    } else if (isWrongColor()){
      shootBall(0.3);
    }

    if(isUpperSlotEmpty()){
      advanceBall();
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
