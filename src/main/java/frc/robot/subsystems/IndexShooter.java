// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

public class IndexShooter extends SubsystemBase {
  //MOTORS
  private WPI_TalonFX topMotor = new WPI_TalonFX(Constants.TOP_INDEX_MOTOR);
  private WPI_TalonFX middleMotor = new WPI_TalonFX(Constants.MIDDLE_INDEX_MOTOR);
  private WPI_TalonFX bottomMotor = new WPI_TalonFX(Constants.BOTTOM_INDEX_MOTOR);

  //COLOR SENSOR
  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private ColorSensorV3 colorSensor = new ColorSensorV3(i2cPort); 
  public ColorMatch colorMatcher = new ColorMatch(); 

  public Color red;
  public Color blue;
  public Color teamColor; 

  //PHOTOELECTRIC SENOR
  //Ultrasonic ultrasonic = new Ultrasonic(1, 2);
  DigitalInput indexSensor = new DigitalInput(0);

  public IndexShooter() {
    red = new Color(1, .5, .5);
    blue = new Color(.5, .5, 1);
    colorMatcher.addColorMatch(red);
    colorMatcher.addColorMatch(blue);
    colorMatcher.setConfidenceThreshold(.75); // Default .95
  }

  /**
   * Sets the team color of the robot for the color sensor to compare to
   * @param color 0 - blue : 1 - red
   */
  public void setColor(int color) {
    teamColor = color == 1 ? blue : red;
  }

  /**
   * Returns the Color detected by the color sensor.
   * @return Color (red, blue, or null)
   */
  public Color getColorMatch(){
    
    Color detectedColor = colorSensor.getColor();
    ColorMatchResult match = colorMatcher.matchClosestColor(detectedColor);

    return match.color;
  }

  /**
   * Returns a boolean depending on if the color of the ball detected is not the same as the robot team color.
   * @return boolean representing if the ball is the same color as the team
   */
  public boolean isCorrectColor(){
    return getColorMatch().equals(teamColor);
  }

  /**
   * Returns a boolean determining if there is a ball in the top slot.
   * @return boolean representing if there is a ball in the top slot
   */
  public boolean isUpperSlotEmpty(){
    return isBottomSlotFilled() && getColorMatch() != red && getColorMatch() != blue;
  }

  /**
   * Sets the speed of the top index motor
   * @param speed percent output from -1 to 1
   */
  public void setTopIndexSpeed(double speed){
    topMotor.set(speed);
  }

  /**
   * Sets the speed of the middle index motor
   * @param speed percent output from -1 to 1
   */
  public void setMiddleIndexSpeed(double speed){
    middleMotor.set(speed);
  }

  /**
   * Sets the speed of the bottom index motor
   * @param speed percent output from -1 to 1
   */
  public void setBottomIndexSpeed(double speed){
    bottomMotor.set(speed);
  }

  /**
   * Returns the if the sensor detecst a ball.
   * @return boolean representing if a ball is present
   */
  public boolean isBottomSlotFilled(){
    return !indexSensor.get();
  }

  @Override
  public void periodic() {
    teamColor = NetworkTableInstance.getDefault().getTable("SmartDashboard").getSubTable("Color").getEntry("active").getString("null").equals("red") ? red : blue;

    SmartDashboard.putNumber("Red", colorSensor.getRed());
    SmartDashboard.putNumber("Green", colorSensor.getGreen());
    SmartDashboard.putNumber("Blue", colorSensor.getBlue());
    SmartDashboard.putBoolean("Boolean - Correct Color", isCorrectColor());
    SmartDashboard.putString("Closest Color", colorMatcher.matchClosestColor(colorSensor.getColor()).color.equals(red) ? "Red" : "Blue");  }
}
