// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class AutonomousTrajectoryRamseteController extends CommandBase {

  private Drivetrain drivetrain;
  private Trajectory trajectory;

  private final Timer timer = new Timer();

  private final RamseteController ramseteController = new RamseteController();

  /** Creates a new AutonomousTrajectoryRamseteController. */
  public AutonomousTrajectoryRamseteController(Drivetrain drivetrain) {
    addRequirements(drivetrain);
    this.drivetrain = drivetrain;    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetrain.resetGyro();
    timer.reset();
    timer.start();

    ramseteController.setEnabled(true);

    trajectory = TrajectoryGenerator.generateTrajectory(
      new Pose2d(0,0, Rotation2d.fromDegrees(0)),
      List.of(
        new Translation2d(1,0)
      ),
      new Pose2d(2,0, Rotation2d.fromDegrees(0)),
      
      new TrajectoryConfig(
        Units.feetToMeters(2),
        Units.feetToMeters(2)
      ).setKinematics(
        new DifferentialDriveKinematics(
          Units.inchesToMeters(21.875)
        )
      )
    );
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double elapsed = timer.get();
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
