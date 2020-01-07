/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import static frc.robot.Constants.DrivetrainConstants.*;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.components.CANEncoderGroup;
import frc.lib.components.TankDrive;

public class Drivetrain extends SubsystemBase {
  
  private final TankDrive drive;
  private final DifferentialDriveKinematics kinematics;
  private final DifferentialDriveOdometry odometry;

  private final CANEncoderGroup leftEncoders;
  private final CANEncoderGroup rightEncoders;

  private final AHRS gyro;

  /**
   * Creates a new Drivetrain.
   */
  public Drivetrain() {
    var left = new SpeedControllerGroup(new CANSparkMax(FRONT_LEFT_ID, MotorType.kBrushless), 
            new CANSparkMax(BACK_LEFT_ID, MotorType.kBrushless));
    var right = new SpeedControllerGroup(new CANSparkMax(FRONT_RIGHT_ID, MotorType.kBrushless), 
            new CANSparkMax(BACK_RIGHT_ID, MotorType.kBrushless));

    drive = new TankDrive(left, right);
    kinematics = new DifferentialDriveKinematics(TRACK_WIDTH);
    odometry = new DifferentialDriveOdometry(new Rotation2d());

    var flEncoder = new CANEncoder(new CANSparkMax(FRONT_LEFT_ID, MotorType.kBrushless));
    var blEncoder = new CANEncoder(new CANSparkMax(BACK_LEFT_ID, MotorType.kBrushless));
    var frEncoder = new CANEncoder(new CANSparkMax(FRONT_RIGHT_ID, MotorType.kBrushless));
    var brEncoder = new CANEncoder(new CANSparkMax(BACK_RIGHT_ID, MotorType.kBrushless));

    leftEncoders = new CANEncoderGroup(flEncoder, blEncoder);
    rightEncoders = new CANEncoderGroup(frEncoder, brEncoder);

    leftEncoders.setDistancePerPulse(2 * Math.PI * WHEEL_RADIUS / ENCODER_RESOLUTION);
    rightEncoders.setDistancePerPulse(2 * Math.PI * WHEEL_RADIUS / ENCODER_RESOLUTION);

    gyro = new AHRS(SPI.Port.kMXP, (byte) 50);
    gyro.reset();
  }

  @Override
  public void periodic() {
    odometry.update(getRotation(), leftEncoders.getPosition(), rightEncoders.getPosition());
  }

  /**
   * @param throttle The foward movement of the robot
   * @param twist The rotation of the robot
   */
  public void arcadeDrive(double throttle, double twist) {
    drive.arcadeDrive(throttle, twist, true);
  }

  /**
   * @param leftVolts Voltage to set left motors to
   * @param rightVolts Voltage to set right motors to
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    drive.tankDriveVolts(leftVolts, rightVolts);
  }

  /**
   * @return the current DifferentialDriveWheelSpeeds
   */
  public DifferentialDriveWheelSpeeds getSpeeds() {
    return new DifferentialDriveWheelSpeeds(leftEncoders.getVelocity(), rightEncoders.getVelocity());
  }

  /**
   * @return current rotation of the drivetrain
   */
  public Rotation2d getRotation() {
    return new Rotation2d(gyro.getAngle());
  }

  /**
   * @return current estimated pose of the drivetrain
   */
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  /**
   * @return the average position of all the encoders
   */
  public double getAverageEncoderDistance() {
    return (leftEncoders.getPosition() + rightEncoders.getPosition()) / 2;
  }

  /**
   * @return the kinematics
   */
  public DifferentialDriveKinematics getKinematics() {
    return kinematics;
  }
}
