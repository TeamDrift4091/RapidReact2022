// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  
  WPI_TalonSRX winchMotor;

  public Climber() {
    winchMotor = new WPI_TalonSRX(Constants.WINCH_MOTOR);
    winchMotor.configFactoryDefault();
  }

  /**
   * Sets the speed of the winch motor.
   * @param speed value between -1 and 1 representing the speed of the motor
   */
  public void moveWinch(double speed){
    winchMotor.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
