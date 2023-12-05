// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
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
import frc.robot.Constants.DriveTrain;

/** Represents a swerve drive style drivetrain. */
public class Drivetrain {
    public static final double kMaxPossibleSpeed = 3.0; // meters per second (could be 3 (was for other robot) when not testing)
    public static final double kMaxAngularSpeed = 3*Math.PI; // 1/2 rotation per second

    private final Translation2d m_frontLeftLocation = new Translation2d(
            DriveTrain.kDistanceMiddleToFrontMotor * DriveTrain.kXForward,
            DriveTrain.kDistanceMiddleToSideMotor * DriveTrain.kYLeft);
    private final Translation2d m_frontRightLocation = new Translation2d(
            DriveTrain.kDistanceMiddleToFrontMotor * DriveTrain.kXForward,
            DriveTrain.kDistanceMiddleToSideMotor * DriveTrain.kYRight);
    private final Translation2d m_backLeftLocation = new Translation2d(
            DriveTrain.kDistanceMiddleToFrontMotor * DriveTrain.kXBackward,
            DriveTrain.kDistanceMiddleToSideMotor * DriveTrain.kYLeft);
    private final Translation2d m_backRightLocation = new Translation2d(
            DriveTrain.kDistanceMiddleToFrontMotor * DriveTrain.kXBackward,
            DriveTrain.kDistanceMiddleToSideMotor * DriveTrain.kYRight);

    private final SwerveModule m_frontLeft = new SwerveModule("FrontLeft", ID.kFrontLeftDrive, ID.kFrontLeftTurn,
            ID.kFrontLeftCANCoder, Offsets.kFrontLeftOffset,
            DriveTrain.turnPID, // p = 6.2
            DriveTrain.drivePID,
            DriveTrain.turnFeedForward,
            DriveTrain.driveFeedForward);
    private final SwerveModule m_frontRight = new SwerveModule("FrontRight", ID.kFrontRightDrive, ID.kFrontRightTurn,
            ID.kFrontRightCANCoder, Offsets.kFrontRightOffset,
            DriveTrain.turnPID, // p = 6.2
            DriveTrain.drivePID,
            DriveTrain.turnFeedForward,
            DriveTrain.driveFeedForward);
    private final SwerveModule m_backLeft = new SwerveModule("BackLeft", ID.kBackLeftDrive, ID.kBackLeftTurn,
            ID.kBackLeftCANCoder,
            Offsets.kBackLeftOffset,
            DriveTrain.turnPID, // p = 6.2
            DriveTrain.drivePID,
            DriveTrain.turnFeedForward,
            DriveTrain.driveFeedForward);
    private final SwerveModule m_backRight = new SwerveModule("BackRight", ID.kBackRightDrive, ID.kBackRightTurn,
            ID.kBackRightCANCoder,
            Offsets.kBackRightOffset,
            DriveTrain.turnPID, // p = 6.2
            DriveTrain.drivePID,
            DriveTrain.turnFeedForward,
            DriveTrain.driveFeedForward);

    private final Pigeon2 m_gyro = new Pigeon2(ID.kGyro);

    /**
     * The order that you initialize these is important! Later uses of functions
     * like toSwerveModuleStates will return the same order that these are provided.
     * See
     * https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/swerve-drive-kinematics.html
     */
    private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
            m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

    private final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
            m_kinematics,
            getGyroYawRotation2d(),
            new SwerveModulePosition[] {
                    m_frontLeft.getPosition(),
                    m_frontRight.getPosition(),
                    m_backLeft.getPosition(),
                    m_backRight.getPosition()
            });

            private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds();
        private Pose2d robotFieldPosition = getRoboPose2d();

    public Drivetrain() {
        // Zero at beginning of match to know what way is forward
        // NOTE: This requires that the robot starts forward
        m_gyro.zeroGyroBiasNow();
        this.resetGyro();
    }

    public void resetGyro() {
        m_gyro.setYaw(0);
    }

    /**
     * Sends swerve info to smart dashboard
     */
    public void print() {
        m_frontLeft.print();
        m_frontRight.print();
        m_backLeft.print();
        m_backRight.print();
    }

    /**
     * Get the yaw of gyro in Rotation2d form
     * 
     * @return chasis angle in Rotation2d
     */
    public Rotation2d getGyroYawRotation2d() {
        return new Rotation2d(Units.degreesToRadians(m_gyro.getYaw()));
    }

    /**
     * Method to drive the robot using joystick info.
     *
     * @param xSpeed        Speed of the robot in the x direction (forward).
     * @param ySpeed        Speed of the robot in the y direction (sideways).
     * @param rotSpeed           Angular rate of the robot.
     * @param fieldRelative Whether the provided x and y speeds are relative to the
     *                      field.
     */
    public void drive(double xSpeed, double ySpeed, double rotSpeed, boolean fieldRelative) {
        SwerveModuleState[] swerveModuleStates = m_kinematics.toSwerveModuleStates(
                fieldRelative
                        ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotSpeed, getGyroYawRotation2d())
                        : new ChassisSpeeds(xSpeed, ySpeed, rotSpeed)); // ADD WHEN WPILIB UPDATE: ChassisSpeeds.fromRobotRelativeSpeeds(xSpeed, ySpeed, rotSpeed, getGyroYawRotation2d());

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxPossibleSpeed);

        // passing back the math from kinematics to the swerves themselves.
        m_frontLeft.setDesiredState(swerveModuleStates[0]);
        m_frontRight.setDesiredState(swerveModuleStates[1]);
        m_backLeft.setDesiredState(swerveModuleStates[2]);
        m_backRight.setDesiredState(swerveModuleStates[3]);

        

        SmartDashboard.putNumber("desired X Speed", xSpeed);
        SmartDashboard.putNumber("desired Y Speed", ySpeed);
        SmartDashboard.putNumber("desired Rot Speed", rotSpeed);
        updateOdometry();
    }

    public Pose2d getRoboPose2d() {
        return m_odometry.getPoseMeters();
    }

    public Pose2d calcRoboPose2dWithVision(double length, double angle1, double angle2) {
        double L = length; //dist between the two april tags
        double a1 = angle1; //angle (from the camera) of the close april tag (a1) and the far april tag (a2)
        double a2 = angle2;
        double gyroOffset = 0;
        double roboAngle = m_gyro.getYaw() + gyroOffset; //angle of the robot (0 degrees = facing the drivers)

        double X = (L * Math.sin(Math.toRadians(90 + roboAngle + a2)) * Math.sin(Math.toRadians(90 - roboAngle - a1)))
                        /Math.sin(Math.toRadians(Math.abs(a2 - a1)));

        double Y = (L * Math.sin(Math.toRadians(90 + roboAngle + a2)) * Math.cos(Math.toRadians(90 - roboAngle - a1)))
                        /Math.sin(Math.toRadians(Math.abs(a2 - a1)));

        return new Pose2d(X, Y, getGyroYawRotation2d());
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

        var frontLeftState = m_frontLeft.getState();
        var frontRightState = m_frontRight.getState();
        var backLeftState = m_backLeft.getState();
        var backRightState = m_backRight.getState();

        // Convert to chassis speeds
        m_chassisSpeeds = m_kinematics.toChassisSpeeds(
        frontLeftState, frontRightState, backLeftState, backRightState);

        // Getting individual speeds
        double forward = m_chassisSpeeds.vxMetersPerSecond;
        double sideways = m_chassisSpeeds.vyMetersPerSecond;
        double angular = m_chassisSpeeds.omegaRadiansPerSecond;
        
        SmartDashboard.putNumber("real X speed", forward);
        SmartDashboard.putNumber("real Y speed", sideways);
        SmartDashboard.putNumber("real Rot speed", Math.toDegrees(angular));
        SmartDashboard.putNumber("real X Pos", robotFieldPosition.getX());
        SmartDashboard.putNumber("real Y Pos", robotFieldPosition.getY());
        SmartDashboard.putNumber("real Rot", robotFieldPosition.getRotation().getDegrees());
    }
}
