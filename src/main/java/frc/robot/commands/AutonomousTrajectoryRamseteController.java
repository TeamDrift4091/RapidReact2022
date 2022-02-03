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
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
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
    System.out.println("Starting Autonomous...");
    drivetrain.resetGyro();
    drivetrain.resetOdometry();
    timer.reset();
    timer.start();

    ramseteController.setEnabled(true);

    trajectory = TrajectoryGenerator.generateTrajectory(
      // Configure path here
      // *********
      new Pose2d(0,0, Rotation2d.fromDegrees(0)),
      List.of(
      ),
      new Pose2d(4,0, Rotation2d.fromDegrees(0)),
      // *********
      
      new TrajectoryConfig(
        Units.feetToMeters(2),
        Units.feetToMeters(2)
      ).setKinematics(
        new DifferentialDriveKinematics(
          Units.inchesToMeters(21.875)
        )
      )
    );

    System.out.println(trajectory.getTotalTimeSeconds());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double elapsed = timer.get();

    Trajectory.State reference = trajectory.sample(elapsed);
    
    ChassisSpeeds speeds = ramseteController.calculate(drivetrain.getPose(), reference);

    var normalized_ramsete_speed = speeds.vxMetersPerSecond / Constants.MAX_SPEED;
    var normalized_ramsete_rotation = speeds.omegaRadiansPerSecond / Constants.MAX_ANGULAR_VELOCITY;

    drivetrain.arcadeDrive(normalized_ramsete_speed, normalized_ramsete_rotation, false);



    var t_pose = reference.poseMeters;
    var t_x = t_pose.getX();
    var t_y = t_pose.getY();
    var t_rotation = t_pose.getRotation().getDegrees();

    var a_pose = drivetrain.getPose();
    var a_x = a_pose.getX();
    var a_y = a_pose.getY();
    var a_rotation = a_pose.getRotation().getDegrees();

    SmartDashboard.putNumber("Ramsete Speed - Normalized", normalized_ramsete_speed);
    SmartDashboard.putNumber("Ramsete Rot - Normalized", normalized_ramsete_rotation);

    SmartDashboard.putNumber("Pose X - Trajectory", t_x);
    SmartDashboard.putNumber("Pose Y - Trajectory", t_y);
    SmartDashboard.putNumber("Pose R - Trajectory", t_rotation);

    SmartDashboard.putNumber("Pose X - Actual", a_x);
    SmartDashboard.putNumber("Pose Y - Actual", a_y);
    SmartDashboard.putNumber("Pose R - Actual", a_rotation);

    SmartDashboard.putNumber("Raw Gyro Angle", drivetrain.getGyroAngle());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Autonomous Complete.");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() > trajectory.getTotalTimeSeconds();
  }
}
