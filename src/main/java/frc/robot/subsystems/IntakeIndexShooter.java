// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeIndexShooter extends SubsystemBase {
  // MOTORS
  // TODO: Should some of these be TalonSRX?
  private WPI_TalonFX shooterMotor = new WPI_TalonFX(Constants.SHOOTER_MOTOR);
  private WPI_TalonFX topIndexMotor = new WPI_TalonFX(Constants.TOP_INDEX_MOTOR);
  private WPI_TalonFX bottomIndexMotor = new WPI_TalonFX(Constants.BOTTOM_INDEX_MOTOR);
  private WPI_TalonFX intakeMotor = new WPI_TalonFX(Constants.INTAKE_MOTOR);

  // SOLENOIDS
  private DoubleSolenoid intakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.INTAKE_SOLENOID_DOWN, Constants.INTAKE_SOLENOID_UP);

  //COLOR SENSOR
  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private ColorSensorV3 colorSensor = new ColorSensorV3(i2cPort); 
  public ColorMatch colorMatcher = new ColorMatch(); 

  public Color red;
  public Color blue;
  public Color blank;
  public Color teamColor; 

  //PHOTOELECTRIC SENOR
  //Ultrasonic ultrasonic = new Ultrasonic(1, 2);
  DigitalInput indexSensor = new DigitalInput(0);

  /** Creates a new IntakeIndexShooter. */
  public IntakeIndexShooter() {
    shooterMotor.configFactoryDefault();
    topIndexMotor.configFactoryDefault();
    bottomIndexMotor.configFactoryDefault();
    intakeMotor.configFactoryDefault();

    // COLOR SENSOR
    red = new Color(1, .5, .5);
    blue = new Color(.5, .5, 1);
    blank = new Color(.5, .5, .5);
    colorMatcher.addColorMatch(red);
    colorMatcher.addColorMatch(blue);
    colorMatcher.setConfidenceThreshold(.75); // Default .95

    //REVERSING INTAKE AND SHOOTER
    intakeMotor.setInverted(true);
    shooterMotor.setInverted(true);

    // SHOOTER
    shooterMotor.setNeutralMode(NeutralMode.Coast);

    // INTAKE
    intakeMotor.setNeutralMode(NeutralMode.Brake);
    intakeSolenoid.set(Value.kReverse); 
  }

  /**
   * Sets the speed of the shooter motor
   * @param speed percent output from -1 to 1
   */
  public void setShooterSpeed(double speed){
    shooterMotor.set(-speed);
  }

  public double getShooterSpeed(){
    return shooterMotor.getSelectedSensorVelocity();
  }

  /**
   * Sets the speed of the top index motor
   * @param speed percent output from -1 to 1
   */
  public void setTopIndexSpeed(double speed){
    topIndexMotor.set(speed);
  }

  public double getTopIndexSpeed(){
    return topIndexMotor.getSelectedSensorVelocity();
  }

  /**
   * Sets the speed of the bottom index motor
   * @param speed percent output from -1 to 1
   */
  public void setBottomIndexSpeed(double speed){
    bottomIndexMotor.set(speed);
  }

  public double getBotomIndexSpeed(){
    return bottomIndexMotor.getSelectedSensorVelocity();
  }

  /**
   * Sets the speed of the intake motor.
   * @param speed percent output from -1 and 1 
   */
  public void setIntakeSpeed(double speed){
    intakeMotor.set(-speed);
  }

  /**
   * Retracts the intake arm to be within the starting bounds of the robot.
   */
  public void retractIntakeArm() {
    intakeSolenoid.set(Value.kForward);
  }

  /**
   * Extends the intake arm to pick up balls.
   */
  public void extendIntakeArm() {
    intakeSolenoid.set(Value.kReverse);
  }

  /**
   * Sets the alliance color of the robot for the color sensor to compare to
   * @param color 0 - blue : 1 - red
   */
  public void setAllianceColor(int color) {
    teamColor = color == 1 ? blue : red;
  }

  /**
   * Returns the Color detected by the color sensor.
   * @return Color (red, blue, or null)
   */
  public Color getColorMatch(){
    if (colorSensor.getProximity() < 170) {
      return blank;
    }
    
    Color detectedColor = colorSensor.getColor();
    ColorMatchResult match = colorMatcher.matchClosestColor(detectedColor);

    return match.color;
  }
  
  /**
   * Returns a boolean depending on if the color of the ball detected is not the same as the robot team color.
   * @return boolean representing if the ball is the same color as the team
   */
  public boolean isCorrectColor(){
    // System.out.println(getColorMatch().equals(red) + ", " + getColorMatch().equals(blue));
    return getColorMatch().equals(teamColor) || getColorMatch().equals(blank);
  }

  /**
   * Returns a boolean determining if there is a ball in the top slot.
   * @return boolean representing if there is a ball in the top slot
   */
  public boolean getUpperSlot(){
    return !getColorMatch().equals(blank);
  }

  /**
   * Returns the if the sensor detecst a ball in the lower slot.
   * @return boolean representing if a ball is present in the lower slot
   */
  public boolean getLowerSlot(){
    return !indexSensor.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    int r = colorSensor.getRed();
    int g = colorSensor.getGreen();
    int b = colorSensor.getBlue();
    int max = Math.max(r,Math.max(g,b));

    try {
    SmartDashboard.putNumber("R", r);
    SmartDashboard.putNumber("G", g);
    SmartDashboard.putNumber("B", b);
    SmartDashboard.putNumber("R (n)", r / max);
    SmartDashboard.putNumber("G (n)", g / max);
    SmartDashboard.putNumber("B (n)", b / max);
    SmartDashboard.putBoolean("isRed", getColorMatch().equals(red));
    SmartDashboard.putBoolean("isBlue", getColorMatch().equals(blue));
    SmartDashboard.putBoolean("isEmpty", getColorMatch().equals(blank));
    } catch (ArithmeticException e) {}
  }
}
