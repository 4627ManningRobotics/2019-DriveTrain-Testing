/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.commands.GTADrive;

/**
 * Add your docs here.
 */
public class DriveTrain extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private TalonSRX left = new TalonSRX(RobotMap.MOTOR_DRIVE_L1);
  private TalonSRX leftFollower = new TalonSRX(RobotMap.MOTOR_DRIVE_L2);
  private TalonSRX right = new TalonSRX(RobotMap.MOTOR_DRIVE_R1);
  private TalonSRX rightFollower = new TalonSRX(RobotMap.MOTOR_DRIVE_R2);

  public void initChassis() {
    leftFollower.set(ControlMode.Follower, RobotMap.MOTOR_DRIVE_L1);
    rightFollower.set(ControlMode.Follower, RobotMap.MOTOR_DRIVE_L1);

  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new GTADrive());
  }

  public void setRampRate(double rate) {
    left.configOpenloopRamp(rate, 0);
    leftFollower.configOpenloopRamp(rate, 0);
    right.configOpenloopRamp(rate, 0);
    rightFollower.configOpenloopRamp(rate, 0);
  }

  public void setLeftMotors(double speed) {
    left.set(ControlMode.PercentOutput, -speed);
  }

  public void setRightMotors(double speed) {
    right.set(ControlMode.PercentOutput, speed);
  }

}
