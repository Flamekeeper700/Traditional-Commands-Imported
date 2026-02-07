// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// TODO: v we need to connect our sparkmaxes v
/*
  import com.revrobotics.spark.SparkBase.PersistMode;
  import com.revrobotics.spark.SparkBase.ResetMode;
  import com.revrobotics.spark.SparkLowLevel.MotorType;
  import com.revrobotics.spark.SparkMax;
  import com.revrobotics.spark.config.SparkMaxConfig;
 */

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

// Class to drive the robot over CAN
public class CANDriveSubsystem extends SubsystemBase {
  private final PWMSparkMax leftLeader;
  private final PWMSparkMax leftFollower;
  private final PWMSparkMax rightLeader;
  private final PWMSparkMax rightFollower;

  private final DifferentialDrive drive;

  public CANDriveSubsystem() {
    // create brushed motors for drive
    leftLeader = new PWMSparkMax(DriveConstants.LEFT_LEADER_ID);
    leftFollower = new PWMSparkMax(DriveConstants.LEFT_FOLLOWER_ID);
    rightLeader = new PWMSparkMax(DriveConstants.RIGHT_LEADER_ID);
    rightFollower = new PWMSparkMax(DriveConstants.RIGHT_FOLLOWER_ID);
    leftLeader.addFollower(leftFollower);
    rightLeader.addFollower(rightFollower);
    // set up differential drive class
    drive = new DifferentialDrive(leftLeader, rightLeader);

    // Set can timeout. Because this project only sets parameters once on
    // construction, the timeout can be long without blocking robot operation. Code
    // which sets or gets parameters during operation may need a shorter timeout.
    // leftLeader.setCANTimeout(250);
    // rightLeader.setCANTimeout(250);
    // leftFollower.setCANTimeout(250);
    // rightFollower.setCANTimeout(250);
  }

  @Override
  public void periodic() {
  }

  // sets the speed of the drive motors
  public void driveArcade(double xSpeed, double zRotation) {
    drive.arcadeDrive(xSpeed, zRotation);
  }
}
