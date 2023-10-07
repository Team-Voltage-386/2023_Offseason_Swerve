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
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ID;
import frc.robot.Constants.Offsets;

/** Represents a swerve drive style drivetrain. */
public class Drivetrain {
  public static final double kMaxSpeed = 0.1; // meters per second (should be 3 when not testing)
  public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second

   private final Translation2d m_frontLeftLocation = new Translation2d(0.365125, -0.263525);
   private final Translation2d m_frontRightLocation = new Translation2d(0.365125, 0.263525);
   private final Translation2d m_backLeftLocation = new Translation2d(-0.365125, -0.263525);
   private final Translation2d m_backRightLocation = new Translation2d(-0.365125, 0.263525);

   private final SwerveModule m_frontLeft = new SwerveModule(ID.kFrontLeftDrive, ID.kFrontLeftTurn, ID.kFrontLeftCANCoder, Offsets.kFrontLeftOffset, false);
   private final SwerveModule m_frontRight = new SwerveModule(ID.kFrontRightDrive, ID.kFrontLeftTurn, ID.kFrontRightCANCoder, Offsets.kFrontRightOffset, false);
   private final SwerveModule m_backLeft = new SwerveModule(ID.kBackLeftDrive, ID.kBackLeftTurn, ID.kBackLeftCANCoder, Offsets.kBackLeftOffset, false);
   private final SwerveModule m_backRight = new SwerveModule(ID.kBackRightDrive, ID.kBackRightTurn, ID.kBackRightCANCoder, Offsets.kBackRightOffset, false);

  private final Pigeon2 m_gyro = new Pigeon2(ID.kGyro);

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
    return new Rotation2d(Units.degreesToRadians(m_gyro.getYaw()));
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
    SmartDashboard.putNumber("Deg Gyro angle", getGyroYawRotation2d().getDegrees());
    SmartDashboard.putNumber("Rad Gyro angle", getGyroYawRotation2d().getRadians());

    SwerveModuleState[] swerveModuleStates =
        m_kinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getGyroYawRotation2d())
                : new ChassisSpeeds(xSpeed, ySpeed, rot));

    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);

    //putting angles of swerve modules on dashboard for debbie's advantage scope and debugging.
    SmartDashboard.putNumber("LF", m_frontLeft.getActualTurningPosition());
    SmartDashboard.putNumber("RF", m_frontRight.getActualTurningPosition());
    SmartDashboard.putNumber("LB", m_backLeft.getActualTurningPosition());
    SmartDashboard.putNumber("RB", m_backRight.getActualTurningPosition());

    //passing back the math from kinematics to the swerves themselves.
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
