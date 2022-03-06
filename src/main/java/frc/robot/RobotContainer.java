// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.Climb;
import frc.robot.commands.IndexShooterCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.JoystickDrive;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.IndexShooter;
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

  // Joystick buttons
  private static JoystickButton joystickButton1 = new JoystickButton(joystick, 1); // Trigger

  // Controller buttons
  private static JoystickButton joystickButton2 = new JoystickButton(controller, 2); // Button 'B'

  // The robot's subsystems and commands are defined here...
  private final Drivetrain drivetrain = new Drivetrain();
  private final IndexShooter indexShooter = new IndexShooter();
  private final Intake intake = new Intake();
  private final Climber climber = new Climber();

  private SendableChooser<Integer> colorChooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // DriverStation.silenceJoystickConnectionWarning(true);

    // Initialize the SmartDashboard choosers
    initializeChoosers();

    drivetrain.setDefaultCommand(new JoystickDrive(
      drivetrain,
      () -> joystick.getY() * -1, // -Y is forward on the joystick
      () -> joystick.getX()
    ));

    climber.setDefaultCommand(new Climb(
      climber,
      () -> controller.getLeftY() * -1
    ));

    indexShooter.setDefaultCommand(
      new IndexShooterCommand(indexShooter, () -> joystickButton1.get())
    );

    intake.setDefaultCommand(new IntakeCommand(
      intake,
      () -> controller.getRightTriggerAxis(),
      () -> controller.getLeftTriggerAxis()
    ));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
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

  /**
   * Anything controlled through the dashboard can be initialized here.
   */
  private void initializeChoosers() {
    // colorChooser
    colorChooser.setDefaultOption("Red", 0);
    colorChooser.addOption("Blue", 1);
    
    SmartDashboard.putData("Color", colorChooser);
    SmartDashboard.putBoolean("Sensor", indexShooter.isBottomSlotFilled());

    // NetworkTableInstance.getDefault().getTable("SmartDashboard").getSubTable("Color").getEntry("active").addListener((event) -> {
    //     indexShooter.setColor(colorChooser.getSelected());
    // }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate); 
  }

}
