// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {

  DifferentialDrive differentialDrive;

  WPI_TalonFX frontLeft;
  WPI_TalonFX middleLeft;
  WPI_TalonFX backLeft;
  WPI_TalonFX frontRight;
  WPI_TalonFX middleRight;
  WPI_TalonFX backRight;

  /** Creates a new Drivetrain. */
  public Drivetrain() {
    frontLeft = new WPI_TalonFX(Constants.FRONT_LEFT_PORT);
    middleLeft = new WPI_TalonFX(Constants.MIDDLE_LEFT_PORT);
    backLeft = new WPI_TalonFX(Constants.BACK_LEFT_PORT);
    frontRight = new WPI_TalonFX(Constants.FRONT_RIGHT_PORT);
    middleRight = new WPI_TalonFX(Constants.MIDDLE_RIGHT_PORT);
    backRight = new WPI_TalonFX(Constants.BACK_RIGHT_PORT);

    differentialDrive = new DifferentialDrive(frontLeft, frontRight);

    middleLeft.follow(frontLeft);
    backLeft.follow(frontLeft);
    middleRight.follow(frontRight);
    backRight.follow(frontRight);

    frontLeft.setInverted(false);
    middleLeft.setInverted(InvertType.FollowMaster);
    backLeft.setInverted(InvertType.FollowMaster);
    frontRight.setInverted(true);
    middleRight.setInverted(InvertType.FollowMaster);
    backRight.setInverted(InvertType.FollowMaster);

    frontLeft.configOpenloopRamp(0.2);
    frontRight.configOpenloopRamp(0.2);

    frontLeft.setNeutralMode(NeutralMode.Brake);
    frontRight.setNeutralMode(NeutralMode.Brake);
  }

  // This this how we will control the robot in most cases
  public void arcadeDrive(double speed, double rotation, boolean squareInputs) {
    differentialDrive.arcadeDrive(speed, rotation, squareInputs);
  }

  /**
   * Sets the left and right motors to a percent output
   * @param leftPercent double
   * @param rightPercent double
   */
  public void set(double leftPercent, double rightPercent){
    frontLeft.set(leftPercent);
    frontRight.set(rightPercent);
  }

  /**
   * Sets the left and right motors to a specified control mode
   * @param controlMode ControlMode
   * @param leftMagnitude double
   * @param rightMagnitude double
   */
  public void set(ControlMode controlMode, double leftMagnitude, double rightMagnitude) {
    frontLeft.set(controlMode, leftMagnitude);
    frontRight.set(controlMode, rightMagnitude);
  }

  /**
   * Sets the encoder values back to zero
   */
  public void resetEncoders(){
    frontLeft.setSelectedSensorPosition(0);
    frontRight.setSelectedSensorPosition(0);
  }

  /**
   * 
   * @return double array of positions [left, right]
   */
  public double[] getPositions(){
    return new double[] {frontLeft.getSelectedSensorPosition(), frontRight.getSelectedSensorPosition()};
  }

  /**
   * 
   * @return double array of velocities [left, right]
   */
  public double[] getVelocities() {
    return new double[] {frontLeft.getSelectedSensorVelocity(), frontRight.getSelectedSensorVelocity()};
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
