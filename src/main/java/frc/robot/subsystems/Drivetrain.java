// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {

  private DifferentialDrive differentialDrive;

  private WPI_TalonFX frontLeft;
  private WPI_TalonFX backLeft;
  private WPI_TalonFX frontRight;
  private WPI_TalonFX backRight;

  private AHRS gyro = new AHRS();

  private final DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(gyro.getRotation2d());

  /** Creates a new Drivetrain. */
  public Drivetrain() {
    frontLeft = new WPI_TalonFX(Constants.FRONT_LEFT_PORT);
    backLeft = new WPI_TalonFX(Constants.BACK_LEFT_PORT);
    frontRight = new WPI_TalonFX(Constants.FRONT_RIGHT_PORT);
    backRight = new WPI_TalonFX(Constants.BACK_RIGHT_PORT);

    frontLeft.configFactoryDefault();
    backLeft.configFactoryDefault();
    frontRight.configFactoryDefault();
    backRight.configFactoryDefault();

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
		frontLeft.configMotionAcceleration(2000, Constants.TIMEOUT_MS);
    frontRight.configMotionCruiseVelocity(3000, Constants.TIMEOUT_MS);
		frontRight.configMotionAcceleration(2000, Constants.TIMEOUT_MS);

    frontLeft.setSelectedSensorPosition(0, 0, Constants.TIMEOUT_MS);
		frontRight.setSelectedSensorPosition(0, 0, Constants.TIMEOUT_MS);
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

  public Pose2d getPose(){
    return odometry.getPoseMeters();
  }

  public void resetOdometry() {
    resetEncoders();
    odometry.resetPosition(new Pose2d(0,0, gyro.getRotation2d()), gyro.getRotation2d());
  }

  public void updateOdometry() {
    Rotation2d rotation2d = gyro.getRotation2d();

    double leftDistance = frontLeft.getSelectedSensorPosition() / Constants.ENCODER_TICKS_PER_FOOT;
    double rightDistance = frontRight.getSelectedSensorPosition() / Constants.ENCODER_TICKS_PER_FOOT;

    odometry.update(rotation2d, leftDistance, rightDistance);
  }

  public void resetEncoders() {
    frontLeft.setSelectedSensorPosition(0);
    frontRight.setSelectedSensorPosition(0);
  }

  public void set(ControlMode controlMode, double leftMagnitude, double rightMagnitude) {
    frontLeft.set(controlMode, leftMagnitude);
    frontRight.set(controlMode, rightMagnitude);
  }

  public double getGyroAngle() {
    return gyro.getAngle();
  }

  public void resetGyro() {
    gyro.reset();
  }
  
  public double[] getEncoderPositions() {
    return new double[] {frontLeft.getSelectedSensorPosition(), frontRight.getSelectedSensorPosition()};
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateOdometry();
  }
}
