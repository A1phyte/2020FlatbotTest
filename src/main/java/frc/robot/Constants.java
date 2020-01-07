/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final class DrivetrainConstants {
    public static final int FRONT_LEFT_ID = 1;
    public static final int FRONT_RIGHT_ID = 2;
    public static final int BACK_LEFT_ID = 3;
    public static final int BACK_RIGHT_ID = 4;

    public static final int NAVX_SPI_PORT = 1;

    public static final double MAX_SPEED = 3.0d; // Meters per second
    public static final double MAX_ACCELERATION = 0.7d; // Meters per second
    public static final double MAX_ROTATION_SPEED = 2 * Math.PI; // Radians per second

    public static final double TRACK_WIDTH = 0.527; // Meters
    public static final double WHEEL_RADIUS = 0.0762; // Meters
    public static final int ENCODER_RESOLUTION = 42; // Ticks per revolution
  }

  public static final class AutoConstants {
    // TODO: Run characterization
    // THESE VALUES WILL NOT WORK
    public static final double S = 0.22; // Volts feedfoward
    public static final double V = 1.98; // Velocity feedfoward
    public static final double A = 0.2; // Acceleration feedfoward

    public static final double P = 8.54; // Proportional feedback
    public static final double I = 0.0; // Integral feedback, leave at 0
    public static final double D = 0.0; // Derivative feedback

    // TODO: Tune
    public static final double B = 2.0; // Ramsete B
    public static final double ZETA = 0.7; // Ramsete Zeta
  }
}
