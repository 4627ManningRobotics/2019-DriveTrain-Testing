/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.SensorTerm;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.team319.follower.FollowsArc;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.commands.GTADrive;

/**
 * Add your docs here.
 */
public class DriveTrain extends Subsystem implements FollowsArc {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private TalonSRX left = new TalonSRX(RobotMap.MOTOR_DRIVE_L1);
  private TalonSRX leftFollower = new TalonSRX(RobotMap.MOTOR_DRIVE_L2);
  private TalonSRX right = new TalonSRX(RobotMap.MOTOR_DRIVE_R1);
  private TalonSRX rightFollower = new TalonSRX(RobotMap.MOTOR_DRIVE_R2);

  private PigeonIMU pigeon = new PigeonIMU(0);

  public void initChassis() {
    leftFollower.set(ControlMode.Follower, RobotMap.MOTOR_DRIVE_L1);
    rightFollower.set(ControlMode.Follower, RobotMap.MOTOR_DRIVE_L1);

    left.setSensorPhase(false);
    right.setSensorPhase(false);
    left.setInverted(true);
    right.setInverted(false);

    left.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
    left.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5, 0);
    right.configRemoteFeedbackFilter(left.getDeviceID(), RemoteSensorSource.TalonSRX_SelectedSensor, 0, 0);
    right.configRemoteFeedbackFilter(pigeon.getDeviceID(), RemoteSensorSource.GadgeteerPigeon_Yaw, 1, 0);

    right.configSensorTerm(SensorTerm.Sum0, FeedbackDevice.RemoteSensor0, 0);
    right.configSensorTerm(SensorTerm.Sum1, FeedbackDevice.QuadEncoder, 0);
    right.configSelectedFeedbackSensor(FeedbackDevice.SensorSum, 0, 0);
    right.configSelectedFeedbackCoefficient(0.5, 0, 0);

    right.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor1, 1, 0);
    right.configSelectedFeedbackCoefficient((3600.0 / 8192.0), 1, 0);

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

  @Override
  public TalonSRX getLeft() {
    return left;
  }

  @Override
  public TalonSRX getRight() {
    return right;
  }

  @Override
  public double getDistance() {
    return right.getSelectedSensorPosition(0);
  }

  @Override
  public Subsystem getRequiredSubsystem() {
    return this;
  }

}
