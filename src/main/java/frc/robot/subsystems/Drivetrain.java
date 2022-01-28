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

    // Motion Magic
    frontLeft.configNominalOutputForward(0, Constants.TIMEOUT_MS);
		frontLeft.configNominalOutputReverse(0, Constants.TIMEOUT_MS);
    frontRight.configNominalOutputForward(0, Constants.TIMEOUT_MS);
		frontRight.configNominalOutputReverse(0, Constants.TIMEOUT_MS);

    frontLeft.configPeakOutputForward(1, Constants.TIMEOUT_MS);
		frontLeft.configPeakOutputReverse(-1, Constants.TIMEOUT_MS);
    frontRight.configPeakOutputForward(1, Constants.TIMEOUT_MS);
		frontRight.configPeakOutputReverse(-1, Constants.TIMEOUT_MS);

    frontLeft.selectProfileSlot(0, 0);
		frontLeft.config_kF(0, .2, Constants.TIMEOUT_MS);
		frontLeft.config_kP(0, 0, Constants.TIMEOUT_MS);
		frontLeft.config_kI(0, 0, Constants.TIMEOUT_MS);
		frontLeft.config_kD(0, .2, Constants.TIMEOUT_MS);
    frontRight.selectProfileSlot(0, 0);
		frontRight.config_kF(0, .2, Constants.TIMEOUT_MS);
		frontRight.config_kP(0, 0, Constants.TIMEOUT_MS);
		frontRight.config_kI(0, 0, Constants.TIMEOUT_MS);
		frontRight.config_kD(0, .2, Constants.TIMEOUT_MS);
    
    frontLeft.configMotionCruiseVelocity(3000, Constants.TIMEOUT_MS);
		frontLeft.configMotionAcceleration(3000, Constants.TIMEOUT_MS);
    frontRight.configMotionCruiseVelocity(3000, Constants.TIMEOUT_MS);
		frontRight.configMotionAcceleration(3000, Constants.TIMEOUT_MS);

    frontLeft.setSelectedSensorPosition(0, 0, Constants.TIMEOUT_MS);
		frontRight.setSelectedSensorPosition(0, 0, Constants.TIMEOUT_MS);

  }

  // This this how we will control the robot in most cases
  public void arcadeDrive(double speed, double rotation, boolean squareInputs) {
    differentialDrive.arcadeDrive(speed, rotation, squareInputs);
  }

  public void set(ControlMode controlMode, double leftMagnitude, double rightMagnitude) {
    frontLeft.set(controlMode, leftMagnitude);
    frontRight.set(controlMode, rightMagnitude);
    differentialDrive.feed();
  }

  public void resetEncoderPosition() {
    frontLeft.setSelectedSensorPosition(0);
    frontRight.setSelectedSensorPosition(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
