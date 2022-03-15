// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.drivetrain.TargetTracking;
import frc.robot.commands.intakeindexshooter.Shoot;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.IntakeIndexShooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutonomousCommand extends SequentialCommandGroup {
  /** Creates a new AutonomousCommand. */
  public AutonomousCommand(Drivetrain drivetrain, IntakeIndexShooter intakeIndexShooter) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new AutonomousTrajectory(
        drivetrain,
        TrajectoryGenerator.generateTrajectory(
          // Configure path here
          // *********
          // Field coords from bottom left
          new Pose2d(0,0, Rotation2d.fromDegrees(0)), // Field coords: (31.5, 9) -45
          List.of(
            // new Translation2d(1.15, .75)
          ),
          new Pose2d(5, 0, Rotation2d.fromDegrees(0)), // Field coords: (49, 5) -45
          // *********
          new TrajectoryConfig(
            Units.feetToMeters(4.5),
            Units.feetToMeters(2)
          ).setKinematics(
            new DifferentialDriveKinematics(
              Constants.WHEEL_BASE_WIDTH
            )
          )

        )
      ),
      new TargetTracking(drivetrain).withTimeout(5),
      new Shoot(intakeIndexShooter),
      new AutonomousTrajectory(
        drivetrain,
        TrajectoryGenerator.generateTrajectory(
          // Configure path here
          // *********
          // Field coords from bottom left
          new Pose2d(0,0, Rotation2d.fromDegrees(0)), // Field coords: (31.5, 9) -45
          List.of(
            // new Translation2d(1.15, .75)
          ),
          new Pose2d(-3, 0, Rotation2d.fromDegrees(0)), // Field coords: (49, 5) -45
          // *********
          new TrajectoryConfig(
            Units.feetToMeters(4.5),
            Units.feetToMeters(2)
          ).setKinematics(
            new DifferentialDriveKinematics(
              Constants.WHEEL_BASE_WIDTH
            )
          ).setReversed(true)

        )
      )
    );
  }
}
