// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.IntakeIndexShooter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.autonomous.Autonomous1Ball;
import frc.robot.commands.autonomous.Autonomous2Ball;
import frc.robot.commands.climber.LowerClimber;
import frc.robot.commands.climber.RaiseClimber;
import frc.robot.commands.drivetrain.JoystickDrive;
import frc.robot.commands.drivetrain.TargetTrackingClockwiseBias;
import frc.robot.commands.drivetrain.TargetTrackingCounterClockwiseBias;
import frc.robot.commands.intakeindexshooter.IntakeIndexShooterCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // the controller will control the intake and climber
  private static XboxController controller = new XboxController(1);
  // private static PS4Controller controller = new PS4Controller(1);

  // Controller buttons
  JoystickButton controllerAButton = new JoystickButton(controller, 1);
  JoystickButton controllerYButton = new JoystickButton(controller, 4);
  JoystickButton controllerLeftBumper = new JoystickButton(controller, 5);
  JoystickButton controllerRightBumper = new JoystickButton(controller, 6);


  // The robot's subsystems and commands are defined here...
  private final Drivetrain drivetrain = new Drivetrain();
  private final IntakeIndexShooter intakeIndexShooter = new IntakeIndexShooter();
  private final Climber climber = new Climber();

  // Choosers
  private final SendableChooser<Command> autonomousChooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    // Initialize all sendable choosers
    initializeChoosers();
    DriverStation.silenceJoystickConnectionWarning(true);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // DRIVETRAIN
    drivetrain.setDefaultCommand(new JoystickDrive(
      drivetrain,
      () -> controller.getLeftY() * -1 * .6, // -Y is forward on the joystick
      () -> controller.getRightX() * controller.getRightX() * Math.signum(controller.getRightX()) * .6
    ));
    controllerRightBumper.whenHeld(new TargetTrackingClockwiseBias(drivetrain));
    controllerLeftBumper.whenHeld(new TargetTrackingCounterClockwiseBias(drivetrain));

    // INTAKE INDEX SHOOTER
    intakeIndexShooter.setDefaultCommand(new IntakeIndexShooterCommand(
      intakeIndexShooter,
      () -> controller.getLeftTriggerAxis() > .1,
      () -> controller.getRightTriggerAxis() > .1
      // () -> controller.getL2Axis() > .1,
      // () -> controller.getR2Axis() > .1
    ));

    // CLIMBER
    controllerAButton.whenHeld(new LowerClimber(climber));
    controllerYButton.whenHeld(new RaiseClimber(climber));
  }

  /**
   * Use this method to create {@link SendableChooser}s and initialize them properly.
   */
  private void initializeChoosers() {
    autonomousChooser.setDefaultOption("2 Ball", new Autonomous2Ball(drivetrain, intakeIndexShooter));
    autonomousChooser.addOption("1 Ball", new Autonomous1Ball(drivetrain, intakeIndexShooter));

    SmartDashboard.putData("Autonomous Chooser", autonomousChooser);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // return new Autonomous2Ball(drivetrain, intakeIndexShooter);
    // return new Autonomous1Ball(drivetrain, intakeIndexShooter);
    return autonomousChooser.getSelected();
  }

  public void updateAllianceColor() {
    boolean isRedAlliance = DriverStation.getAlliance().equals(DriverStation.Alliance.Red);
    intakeIndexShooter.setAllianceColor(isRedAlliance ? 0 : 1);
  }
}
