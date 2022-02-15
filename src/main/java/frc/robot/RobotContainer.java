// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.JoystickDrive;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // The robot's controllers...
  // the joystick will control the drivetrain and shooter subsystems
  private static Joystick joystick = new Joystick(0);
  // the controller will control the intake and climber
  private static XboxController controller = new XboxController(1);

  // The robot's buttons...
  private static JoystickButton joystickButton1 = new JoystickButton(joystick, 1); // Trigger

  private static JoystickButton controllerButton2 = new JoystickButton(controller, 2); // Button 'B'

  // The robot's subsystems and commands are defined here...
  private final Drivetrain drivetrain = new Drivetrain();

  private final Intake intake = new Intake();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    drivetrain.setDefaultCommand(new JoystickDrive(
      drivetrain,
      () -> joystick.getY() * -1, // -Y is forward on the joystick
      () -> joystick.getX()
    ));

    intake.setDefaultCommand(new IntakeCommand(
      intake,
      () -> controller.getRightTriggerAxis(),
      () -> controller.getLeftTriggerAxis()
      // () -> (controller.getLeftTriggerAxis() + controller.getRightTriggerAxis() > .1) // default position of either trigger is 0
      // ** OR **
      // () -> (joystickButton1.get())
    ));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }
}
