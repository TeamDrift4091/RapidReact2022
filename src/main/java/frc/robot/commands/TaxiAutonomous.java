// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;

public class TaxiAutonomous extends SequentialCommandGroup {
  /** Add your docs here. */

  public TaxiAutonomous(Drivetrain drivetrain) {
    addCommands(DriveDistanceAutomous(drivetrain, 84.75)); // 84.75 inches
  }
}
