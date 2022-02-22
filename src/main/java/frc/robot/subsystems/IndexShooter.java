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

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.util.Color;

public class IndexShooter extends SubsystemBase {
  /** Creates a new IndexShooter. */

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

  //ULTRASONIC SENOR
  Ultrasonic ultrasonic = new Ultrasonic(1, 2);

  public IndexShooter() {
    red = new Color(1, 1, 1);
    blue = new Color(1, 1, 1);
    colorMatcher.addColorMatch(red);
    colorMatcher.addColorMatch(blue);
    //colorMatcher.setConfidenceThreshold(.95);
  }

  public void setColor(int color){
    teamColor = color == 1 ? blue : red;
  }

  public Color getColorMatch(){
    
    Color detectedColor = colorSensor.getColor();
    ColorMatchResult match = colorMatcher.matchClosestColor(detectedColor);

    return match.color;
  }

  public void setTopIndexSpeed(double speed){
    topMotor.set(speed);
  }

  public void setMiddleIndexSpeed(double speed){
    middleMotor.set(speed);
  }

  public void setBottomIndexSpeed(double speed){
    bottomMotor.set(speed);
  }

  public double getRangeInches(){
    return ultrasonic.getRangeInches();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
