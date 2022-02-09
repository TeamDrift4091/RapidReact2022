// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {

  private DifferentialDrive differentialDrive;

  private WPI_TalonFX frontLeft;
  private WPI_TalonFX middleLeft;
  private WPI_TalonFX backLeft;
  private WPI_TalonFX frontRight;
  private WPI_TalonFX middleRight;
  private WPI_TalonFX backRight;

  private AHRS gyro = new AHRS();

  private final DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(gyro.getRotation2d());

  // DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(21.875));
  // DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(getHeading());

  // PIDController leftPidController = new PIDController(9.95, 0, 0);
  // PIDController rightPidController = new PIDController(9.95, 0, 0);
  // SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.268, 1.89, 0.243); // NEED TO UPDATE VALUES USING SOFTWARE

  /** Creates a new Drivetrain. */
  public Drivetrain() {
    frontLeft = new WPI_TalonFX(Constants.FRONT_LEFT_PORT);
    middleLeft = new WPI_TalonFX(Constants.MIDDLE_LEFT_PORT);
    backLeft = new WPI_TalonFX(Constants.BACK_LEFT_PORT);
    frontRight = new WPI_TalonFX(Constants.FRONT_RIGHT_PORT);
    middleRight = new WPI_TalonFX(Constants.MIDDLE_RIGHT_PORT);
    backRight = new WPI_TalonFX(Constants.BACK_RIGHT_PORT);

    differentialDrive = new DifferentialDrive(frontLeft, frontRight);
    differentialDrive.setSafetyEnabled(false);

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

  // public Rotation2d getHeading(){
  //   return Rotation2d.fromDegrees(-gyro.getAngle());
  // }

// public PIDController getLeftPIDController(){
//   return leftPidController;
// }

// public PIDController getrightPIDController(){
//   return rightPidController;
// }

// public DifferentialDriveKinematics getKinematics(){
//   return kinematics;  
// }

// public DifferentialDriveWheelSpeeds getWheelSpeeds(){
  
//   return new DifferentialDriveWheelSpeeds(
//     frontLeft.getSelectedSensorVelocity() / 7.29 * 2 * Math.PI * Units.inchesToMeters(3.0) / 60,
//     frontRight.getSelectedSensorVelocity() / 7.29 * 2 * Math.PI * Units.inchesToMeters(3.0) / 60
//     );
// }

  // public void setOutputByVoltage(double leftVolts, double rightVolts) {
  //   System.out.println(leftVolts / 12.);
  //   System.out.println(rightVolts / 12. + "\n");
  //   frontLeft.set(leftVolts / 12.);
  //   frontRight.set(rightVolts / 12.);
  // }

  public double getGyroAngle() {
    return gyro.getAngle();
  }

  public void resetGyro() {
    gyro.reset();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // pose = odometry.update(getHeading(), frontLeft.getSelectedSensorPosition(), frontRight.getSelectedSensorPosition());
    updateOdometry();

    SmartDashboard.putNumber("RPM (frontLeft)", frontLeft.getSelectedSensorVelocity()*600/4096.);
    SmartDashboard.putNumber("RPM (frontRight)", frontRight.getSelectedSensorVelocity()*600/4096.);
    if (SmartDashboard.getNumber("RPM (Average)", 0) < (frontLeft.getSelectedSensorVelocity()+frontRight.getSelectedSensorVelocity())*600/4096./2.) {
      SmartDashboard.putNumber("RPM (Average)", (frontLeft.getSelectedSensorVelocity()+frontRight.getSelectedSensorVelocity())*600/4096./2.);
    }
    SmartDashboard.putNumber("MPS (frontLeft)", Units.feetToMeters(((frontLeft.getSelectedSensorVelocity()/60.)/7.6)/(.5*Math.PI)));
    SmartDashboard.putNumber("MPS (frontRight)", Units.feetToMeters(((frontRight.getSelectedSensorVelocity()/60.)/7.6)/(.5*Math.PI)));
    SmartDashboard.putNumber("MPS (Average)", (Units.feetToMeters(((frontLeft.getSelectedSensorVelocity()/60.)/7.6)/(.5*Math.PI)) +
      Units.feetToMeters(((frontRight.getSelectedSensorVelocity()/60.)/7.6)/(.5*Math.PI)))/2.);

    SmartDashboard.putNumber("Encoder Ticks (frontLeft)", frontLeft.getSelectedSensorPosition());
  }
}
