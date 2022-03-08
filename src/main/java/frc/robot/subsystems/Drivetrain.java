// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {

  DifferentialDrive differentialDrive;

  WPI_TalonFX frontLeft;
  WPI_TalonFX backLeft;
  WPI_TalonFX frontRight;
  WPI_TalonFX backRight;

  /** Creates a new Drivetrain. */
  public Drivetrain() {
    frontLeft = new WPI_TalonFX(Constants.FRONT_LEFT_PORT);
    backLeft = new WPI_TalonFX(Constants.BACK_LEFT_PORT);
    frontRight = new WPI_TalonFX(Constants.FRONT_RIGHT_PORT);
    backRight = new WPI_TalonFX(Constants.BACK_RIGHT_PORT);

    differentialDrive = new DifferentialDrive(frontLeft, frontRight);
    differentialDrive.setSafetyEnabled(false);

    backLeft.follow(frontLeft);
    backRight.follow(frontRight);

    frontLeft.setInverted(false);
    backLeft.setInverted(InvertType.FollowMaster);
    frontRight.setInverted(true);
    backRight.setInverted(InvertType.FollowMaster);

    frontLeft.configOpenloopRamp(0.2);
    frontRight.configOpenloopRamp(0.2);

    frontLeft.setNeutralMode(NeutralMode.Brake);
    frontRight.setNeutralMode(NeutralMode.Brake);
  }

  /**
   * Arcade drive method using differential drive.
   *
   * @param speed The robot's speed along the X axis [-1.0..1.0]. Forward is positive.
   * @param rotation The robot's rotation rate around the Z axis [-1.0..1.0]. Clockwise is
   *     positive.
   * @param squareInputs If set, decreases the input sensitivity at low speeds.
   */
  public void arcadeDrive(double speed, double rotation, boolean squareInputs) {
    differentialDrive.arcadeDrive(speed, rotation, squareInputs);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
