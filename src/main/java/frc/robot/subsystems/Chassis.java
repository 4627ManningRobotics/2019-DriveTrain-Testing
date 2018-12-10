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

/**
 * Add your docs here.
 */
public class Chassis extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private TalonSRX motorL1 = new TalonSRX(RobotMap.MOTOR_DRIVE_L1);
  private TalonSRX motorL2 = new TalonSRX(RobotMap.MOTOR_DRIVE_L2);
  private TalonSRX motorR1 = new TalonSRX(RobotMap.MOTOR_DRIVE_R1);
  private TalonSRX motorR2 = new TalonSRX(RobotMap.MOTOR_DRIVE_R2);

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void setRampRate(double rate) {
    motorL1.configOpenloopRamp(rate, 0);
    motorL2.configOpenloopRamp(rate, 0);
    motorR1.configOpenloopRamp(rate, 0);
    motorR2.configOpenloopRamp(rate, 0);
  }

  public void setLeftMotors(double speed) {
    motorL1.set(ControlMode.PercentOutput, -speed);
    motorL2.set(ControlMode.PercentOutput, -speed);

  }

  public void setRightMotors(double speed) {
    motorR1.set(ControlMode.PercentOutput, speed);
    motorR2.set(ControlMode.PercentOutput, speed);
  }

}
