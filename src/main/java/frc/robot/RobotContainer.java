// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.JoystickDrive;
import frc.robot.commands.TankDrive;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private enum DriveMode {
    TANK_DRIVE,
    ARCADE_DRIVE
  }
  private SendableChooser<DriveMode> driveModeChooser = new SendableChooser<>();

  // The robot's controllers...
  // the joystick will control the drivetrain and shooter subsystems
  private static Joystick joystick = new Joystick(0);
  // the controller will control the intake and climber
  private static XboxController controller = new XboxController(1);

  // The robot's subsystems and commands are defined here...
  private final Drivetrain drivetrain = new Drivetrain();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // drivetrain.setDefaultCommand(new JoystickDrive(
    //   drivetrain,
    //   () -> joystick.getY() * -1, // -Y is forward on t he joystick
    //   () -> joystick.getX() * .75 // Decrease the sensitivity
    // ));

    // StringBuilder stringBuilder = new StringBuilder(">>>");
    // stringBuilder.append(drivetrain).append(joystick.getX()).append(joystick.getY()).append(controller.getLeftY()).append(controller.getRightY()).append(driveModeChooser.getSelected());
    // System.out.println(stringBuilder);

    drivetrain.setDefaultCommand(
      new ConditionalCommand(
        new JoystickDrive(
          drivetrain,
          () -> joystick.getY(),
          () -> joystick.getX()
        ), new TankDrive(
          drivetrain,
          () -> controller.getLeftY(),
          () -> controller.getRightY()
        ),
        () -> driveModeChooser.getSelected() == DriveMode.ARCADE_DRIVE
      )
    );
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    driveModeChooser.setDefaultOption("Tank Drive", DriveMode.TANK_DRIVE);
    driveModeChooser.addOption("Arcade Drive", DriveMode.ARCADE_DRIVE);
    SmartDashboard.putData(driveModeChooser);
  }

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
