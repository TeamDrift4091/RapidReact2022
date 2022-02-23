// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class Intake extends SubsystemBase {
  /** Creates a new Intake. */

  private DoubleSolenoid leftSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.INTAKE_DOWN_LEFT, Constants.INTAKE_UP_LEFT);
  private DoubleSolenoid rightSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.INTAKE_DOWN_RIGHT, Constants.INTAKE_UP_RIGHT);
  private WPI_TalonFX intakeMotor = new WPI_TalonFX(Constants.INTAKE_PORT);


  public Intake() {
    intakeMotor.setNeutralMode(NeutralMode.Brake);
    leftSolenoid.set(Value.kReverse);
    rightSolenoid.set(Value.kReverse);
    
  }

  /**
   * Sets the speed of the intake motor.
   * @param speed value between -1 and 1 
   */
  public void setIntakeSpeed(double speed){
    intakeMotor.set(speed);
  }

  /**
   * Raises the intake arm to be within the starting bounds of the robot.
   */
  public void raiseIntakeArm() {
    leftSolenoid.set(Value.kForward);
    rightSolenoid.set(Value.kForward);
  }

  /**
   * Lowers the intake arm to pick up balls.
   */
  public void lowerIntakeArm() {
    // System.out.println("LowerIntakeArm is called");
    leftSolenoid.set(Value.kReverse);
    rightSolenoid.set(Value.kReverse);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }
}
