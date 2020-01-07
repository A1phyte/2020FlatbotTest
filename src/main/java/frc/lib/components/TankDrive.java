/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.lib.components;

import edu.wpi.first.wpilibj.SpeedController;

/**
 * Add your docs here.
 */
public class TankDrive {

  private final SpeedController left;
  private final SpeedController right;

  private static final double DEADBAND = 0.1;

  public TankDrive(SpeedController left, SpeedController right) {
    this.left = left;
    this.right = right;
  }

  public void tankDrive(double leftSpeed, double rightSpeed) {
    left.set(leftSpeed);
    right.set(rightSpeed);
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    left.setVoltage(leftVolts);
    right.setVoltage(rightVolts);
  }

  public void arcadeDrive(double throttle, double twist, boolean squareInputs) {
    throttle = clamp(throttle, -1.0, 1.0);
    twist = clamp(twist, -1.0, 1.0);

    throttle = applyDeadband(throttle, DEADBAND);
    twist = applyDeadband(twist, DEADBAND);

    if (squareInputs) {
      throttle = Math.copySign(throttle * throttle, throttle);
      twist = Math.copySign(twist * twist, twist);
    }

    tankDrive(throttle + twist, throttle - twist);
  }

  private double applyDeadband(double val, double deadband) {
    return Math.abs(val) <= deadband ? 0 : val;
  }

  private double clamp(double val, double min, double max) {
    val = val < min ? min : val;
    val = val > max ? max : val;
    return val;
  }
}
