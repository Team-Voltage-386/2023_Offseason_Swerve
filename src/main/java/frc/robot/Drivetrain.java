// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.AnalogGyro;

/** Represents a swerve drive style drivetrain. */
public class Drivetrain {
  public static final double kMaxSpeed = 3.0; // 3 meters per second
  public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second

   private final Translation2d m_frontLeftLocation = new Translation2d(0.365125, -0.263525);
   private final Translation2d m_frontRightLocation = new Translation2d(0.365125, 0.263525);
   private final Translation2d m_backLeftLocation = new Translation2d(-0.365125, -0.263525);
   private final Translation2d m_backRightLocation = new Translation2d(-0.365125, 0.263525);

   private final SwerveModule m_frontLeft = new SwerveModule(18, 14, 24, 84.03);
   private final SwerveModule m_frontRight = new SwerveModule(15, 11, 21, 210.05);
   private final SwerveModule m_backLeft = new SwerveModule(17, 13, 23, 68.05);
   private final SwerveModule m_backRight = new SwerveModule(16, 12, 22, 43.2);

  private final Pigeon2 m_gyro = new Pigeon2(2);

  private final SwerveDriveKinematics m_kinematics =
      new SwerveDriveKinematics(
          m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

  private final SwerveDriveOdometry m_odometry =
      new SwerveDriveOdometry(
          m_kinematics,
          getGyroYawRotation2d(),
          new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_backLeft.getPosition(),
            m_backRight.getPosition()
          });

  public Drivetrain() {
    // m_gyro.reset();
    m_gyro.zeroGyroBiasNow();
  }


  /**
   * returns yaw of gyro in Rotation2d form
   * @return chasis angle in Rotation2d
   */
  public Rotation2d getGyroYawRotation2d() {
    return new Rotation2d(m_gyro.getYaw());
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    System.out.printf("Gyro yaw: %f\n", m_gyro.getYaw());

    var swerveModuleStates =
        m_kinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getGyroYawRotation2d())
                : new ChassisSpeeds(xSpeed, ySpeed, rot));

    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);

    for (int i = 0; i < swerveModuleStates.length; i++) {
        System.out.printf("State %d: %s\n", i, swerveModuleStates[i].toString());
    }

    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);
  }

  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    m_odometry.update(
        getGyroYawRotation2d(),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_backLeft.getPosition(),
          m_backRight.getPosition()
        });
  }
}
