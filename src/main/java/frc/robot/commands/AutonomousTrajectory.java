// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
//import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.subsystems.Drivetrain;

public class AutonomousTrajectory extends CommandBase {
  /** Creates a new AutonomousTrajectory. */

  Drivetrain drivetrain = new Drivetrain();
  
  //Timer trajectoryTimer = new Timer();

  RamseteCommand ramseteCommand;

  TrajectoryConfig config;
  Trajectory trajectory;

  public AutonomousTrajectory(Drivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //trajectoryTimer.start();
    
    TrajectoryConfig config = new TrajectoryConfig(Units.feetToMeters(2), Units.feetToMeters(2));
    config.setKinematics(drivetrain.getKinematics());
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, new Rotation2d(0)),
    // Pass through these two interior waypoints, making an 's' curve path
    List.of(
        new Translation2d(1, 0)
    ),
    // End 3 meters straight ahead of where we started, facing forward
    new Pose2d(1, 0, new Rotation2d(0)),
    // Pass config
    config);  

    RamseteCommand ramseteCommand =
        new RamseteCommand(
            trajectory,
           drivetrain::getPose,
            new RamseteController(2.0, 0.7),
            drivetrain.getFeedforward(),
            drivetrain.getKinematics(),
            drivetrain::getWheelSpeeds,
            drivetrain.getLeftPIDController(),
            drivetrain.getrightPIDController(),
            // RamseteCommand passes volts to the callback
            drivetrain::setOutput,
            drivetrain);

    ramseteCommand.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ramseteCommand.execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ramseteCommand.end(interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ramseteCommand.isFinished();
    //trajectoryTimer.get() > trajectory.getTotalTimeSeconds();
  }
}
