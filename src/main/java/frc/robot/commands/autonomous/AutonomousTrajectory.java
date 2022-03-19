// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class AutonomousTrajectory extends CommandBase {

  private Drivetrain drivetrain;
  private Trajectory trajectory;

  private final Timer timer = new Timer();

  private final RamseteController ramseteController = new RamseteController();

  /** Creates a new AutonomousTrajectoryRamseteController. */
  public AutonomousTrajectory(Drivetrain drivetrain, Trajectory trajectory) {
    addRequirements(drivetrain);
    this.drivetrain = drivetrain;    

    this.trajectory = trajectory;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetrain.resetGyro();
    drivetrain.resetOdometry();
    timer.reset();
    timer.start();

    ramseteController.setEnabled(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double elapsed = timer.get();

    Trajectory.State reference = trajectory.sample(elapsed);
    
    ChassisSpeeds speeds = ramseteController.calculate(drivetrain.getPose(), reference);

    double normalized_ramsete_speed = speeds.vxMetersPerSecond / Constants.MAX_SPEED;
    double normalized_ramsete_rotation = -speeds.omegaRadiansPerSecond / Constants.MAX_ANGULAR_VELOCITY;

    drivetrain.arcadeDrive(normalized_ramsete_speed, normalized_ramsete_rotation, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() > trajectory.getTotalTimeSeconds();
  }
}
