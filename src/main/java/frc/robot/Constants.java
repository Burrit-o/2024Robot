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
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
  }

  public class IPFSConstants {
    public static final int TLShooterMotor = 50;
    public static final int TRShooterMotor = 51;
    public static final int BLShooterMotor = 52;
    public static final int BRShooterMotor = 53;
    public static final int PickupMotorTop = 56;
    public static final int PickupMotorBottom = 57;
    public static final int LFeederMotor = 58;
    public static final int RFeederMotor = 59;
  }

  public class LiftConstants {
    public static final int ToFSensor = 62;
    public static final int BackupToFSensor = 61;
    public static final int LeftLiftMotor = 54;
    public static final int RightLiftMotor = 55;

    //Setpoint Heights, totally random at the moment, once tested will be replaced
    public static final double ClimbTop = 500;
    public static final double ClimbBottom = 100;
    public static final double AmpHeight = 450;
    public static final double SpeakerHeight = 300;
    public static final double Short = 50;
    public static final double PickupHeight = 50;
    public static final double Stow = 150;
    public static final double defaultStartingHeight = Stow;
  }
}

