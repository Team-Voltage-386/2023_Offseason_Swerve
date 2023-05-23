// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.WPI_SwerveModule;;

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
  }

  public static final class AutoConstants {

    public static final HashMap<String, Command> kEventMap = new HashMap<>();

    public static final double driveTolerance = 0.2;
    public static final double headingTolerance = 90;

    public static final double[] kAutoPositionPID = { .3, .075, .01 };
    public static final double[] kAutoHeadingPID = { 0.01, 0, 0 };
  }

   /** the indexes to address buttons on the controller */
   public static final class ControllerConstants {
    public static final double kDeadband = .1;
    public static final double kJoyStickDeadzone = 0.02;
    public static final Joystick kDriver = new Joystick(0);
    public static final Joystick kManipulator = new Joystick(1);

    public static final int kLeftVertical = 1;
    public static final int kRightVertical = 5;
    public static final int kLeftHorizontal = 0;
    public static final int kRightHorizontal = 4;
    public static final int kLeftTrigger = 2;
    public static final int kRightTrigger = 3;

    public static final int kA = 1;
    public static final int kB = 2;
    public static final int kX = 3;
    public static final int kY = 4;
    public static final int kLeftBumper = 5;
    public static final int kRightBumper = 6;
    public static final int kLeftOptions = 7;
    public static final int kRightOptions = 8;
    public static final int kLeftJoystickPressed = 9;
    public static final int kRightJoystickPressed = 10;
  }

  /** Can IDs, PID values, ect. */
        public static final class DriveConstants {

                public static final double kMaxRotSpeed = 180;
                public static final double kMaxDriveSpeed = 4;
                public static final double kMaxDriveAccel = 3;
                public static final double kMaxRotAccel = 120;
                public static final double kSlowRotSpeed = 45;
                public static final double kSlowDriveSpeed = 1;
                public static final int kIMUid = 2;
                public static final double[] kSwerveSteerPID = { 0.006, 0.01, 0.0 }; // 0.01,0.0,0.001
                public static final double[] kSwerveDrivePID = { 0.3, 2, 1 }; // 0.35,2,0.01
                public static final double kSwerveDriveEncConv = 0.000745;

                //WPISwerves
                public static final WPI_SwerveModule LeftFrontWPI = new WPI_SwerveModule(14, 18, kSwerveDriveEncConv,
                                kSwerveSteerPID,
                                kSwerveDrivePID, 24, 84.03, "LF");
                public static final WPI_SwerveModule RightFrontWPI = new WPI_SwerveModule(11, 15, kSwerveDriveEncConv,
                                kSwerveSteerPID,
                                kSwerveDrivePID, 21, 210.05, "RF");
                public static final WPI_SwerveModule LeftRearWPI = new WPI_SwerveModule(13, 17, kSwerveDriveEncConv,
                                kSwerveSteerPID,
                                kSwerveDrivePID, 23, 68.05, "LR");// faulty encoder offset
                public static final WPI_SwerveModule RightRearWPI = new WPI_SwerveModule(12, 16, kSwerveDriveEncConv,
                                kSwerveSteerPID,
                                kSwerveDrivePID, 22, 43.2, "RR");

                //WPI SwerveDriveKineMatics
                public static final double kWheelRadiusMeters = Units.inchesToMeters(2);
                //Wheel Radius in Meters (2 inches)
                public static final double kDistBetweenLeftRightWheels = 0.52705;
                // Distance between right and left wheels
                public static final double kDistBetweenFrontBackWheels = 0.73025;
                // Distance between front and back wheels
                
                //FOUND IT !!!!!!!!!!!!!!!!!!!!!!!!!!
                //LEFTFRONT, RIGHTFRONT, LEFTBACK, RIGHTBACK
                public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(-kDistBetweenLeftRightWheels / 2, kDistBetweenFrontBackWheels / 2),
                new Translation2d(kDistBetweenLeftRightWheels / 2, kDistBetweenFrontBackWheels / 2),
                new Translation2d(-kDistBetweenLeftRightWheels / 2, -kDistBetweenFrontBackWheels / 2),
                new Translation2d(kDistBetweenLeftRightWheels / 2, -kDistBetweenFrontBackWheels / 2));

        }
}
