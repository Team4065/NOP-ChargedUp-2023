// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int Joystick = 0;
    public static final int RightMaster = 2;
    public static final int RightSlave = 4;
    public static final int LeftMaster = 1;
    public static final int LeftSlave = 3;
    public static final int elevator = 0;
  }

  public static final class DriveTrainConstants {
    public static final double Speed1 = 1.0;
    public static final double Speed2 = 0.5;
    public static final double Speed3 = 0.2;
  }

public static final int Intake = 0;

}
