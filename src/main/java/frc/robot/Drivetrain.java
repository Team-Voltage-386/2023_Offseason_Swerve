// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.sensors.Pigeon2;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.ID;
import frc.robot.Constants.Offsets;
import frc.robot.Constants.DriveTrain;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Represents a swerve drive style drivetrain. */
public class Drivetrain extends SubsystemBase{
    public static final double kMaxSpeed = 2.0; // meters per second (could be 3 (was for other robot) when not testing)
    public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second

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
            new double[] { 1.0, 1.0,
                    0.0 }, // 0.1, 0.5, 0.008
            new double[] { 0.1, 0.5, 0.008 });
    private final SwerveModule m_frontRight = new SwerveModule("FrontRight", ID.kFrontRightDrive, ID.kFrontRightTurn,
            ID.kFrontRightCANCoder, Offsets.kFrontRightOffset,
            new double[] { 1.0, 1.0,
                    0.0 },
            new double[] { 0.1, 0.5, 0.008 });
    private final SwerveModule m_backLeft = new SwerveModule("BackLeft", ID.kBackLeftDrive, ID.kBackLeftTurn,
            ID.kBackLeftCANCoder,
            Offsets.kBackLeftOffset,
            new double[] { 1.0, 1.0,
                    0.0 },
            new double[] { 0.1, 0.5, 0.008 });
    private final SwerveModule m_backRight = new SwerveModule("BackRight", ID.kBackRightDrive, ID.kBackRightTurn,
            ID.kBackRightCANCoder, Offsets.kBackRightOffset,
            new double[] { 1.0, 1.0,
                    0.0 },
            new double[] { 0.1, 0.5, 0.008 });

    private final Pigeon2 m_gyro = new Pigeon2(ID.kGyro);

    /**
     * The order that you initialize these is important! Later uses of functions
     * like toSwerveModuleStates will return the same order that these are provided.
     * See
     * https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/swerve-drive-kinematics.html
     */
    public final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
            m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

    public final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
            m_kinematics,
            getGyroYawRotation2d(),
            getModulePositions());

    public ChassisSpeeds m_chassisSpeeds = m_kinematics.toChassisSpeeds(getModuleStates());

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
     * @param rot           Angular rate of the robot.
     * @param fieldRelative Whether the provided x and y speeds are relative to the
     *                      field.
     */
    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        SwerveModuleState[] swerveModuleStates = m_kinematics.toSwerveModuleStates(
                fieldRelative
                        ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getGyroYawRotation2d())
                        : new ChassisSpeeds(xSpeed, ySpeed, rot));

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);

        // passing back the math from kinematics to the swerves themselves.
        m_frontLeft.setDesiredState(swerveModuleStates[0]);
        m_frontRight.setDesiredState(swerveModuleStates[1]);
        m_backLeft.setDesiredState(swerveModuleStates[2]);
        m_backRight.setDesiredState(swerveModuleStates[3]);
        updateChassispeeds();
    }

        // Assuming this method is part of a drivetrain subsystem that provides the necessary methods
        // https://github.com/HighlanderRobotics/Rapid-React/blob/main/src/main/java/frc/robot/subsystems/DrivetrainSubsystem.java
        public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath) {
                return new SequentialCommandGroup(
                        new InstantCommand(() -> {
                        // Reset odometry for the first path you run during auto
                        if(isFirstPath){
                                this.resetOdometry(traj.getInitialHolonomicPose());
                        }
                        }),
                        new PPSwerveControllerCommand(
                                traj, 
                                () -> m_odometry.getPoseMeters(), // Pose supplier
                                this.m_kinematics, // SwerveDriveKinematics
                                new PIDController(0.5, 0, 0), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
                                new PIDController(0.5, 0, 0), // Y controller (usually the same values as X controller)
                                new PIDController(0.5, 0, 0), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
                                (SwerveModuleState[] states) -> { // Consumes the module states to set the modules moving in the directions we want
                                        m_chassisSpeeds = m_kinematics.toChassisSpeeds(states);
                                        setModuleStates(states);
                                }, // Module states consumer
                                true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
                                this // Requires this drive subsystem
                        )
                );
        }

        public void setModuleStates(SwerveModuleState[] swerveModuleStates) {
                m_frontLeft.setDesiredState(swerveModuleStates[0]);
                m_frontRight.setDesiredState(swerveModuleStates[1]);
                m_backLeft.setDesiredState(swerveModuleStates[2]);
                m_backRight.setDesiredState(swerveModuleStates[3]);
        }

        public SwerveModuleState[] getModuleStates() {
                return new SwerveModuleState[] {
                        m_frontLeft.getState(),
                        m_frontRight.getState(),
                        m_backLeft.getState(),
                        m_backRight.getState()
                };
        }

        public void updateChassispeeds() {
                m_chassisSpeeds = m_kinematics.toChassisSpeeds(getModuleStates());
        }

    /**
     * resets odometry using the current rotation, the current module positions, and a Pose2d
     * @param roboPose
     */
    public void resetOdometry(Pose2d roboPose) {
        m_odometry.resetPosition(getGyroYawRotation2d(), getModulePositions(), roboPose);
    }

    /**
     * returns swerve module positions
     * @return
     */
    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
                m_frontLeft.getPosition(),
                m_frontRight.getPosition(),
                m_backLeft.getPosition(),
                m_backRight.getPosition()
        };
    }

    public Pose2d calcRoboPose2dWithVision() {
        double L = 1; //dist between the two april tags
        double a1 = 15; //placeholder values, angle (from the camera) of the close april tag (a1) and the far april tag (a2)
        double a2 = 20;
        double roboAngle = m_gyro.getYaw(); //angle of the robot (0 degrees = facing the drivers)

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
                getModulePositions());
        if(Math.abs(m_gyro.getPitch()) > 4 || Math.abs(m_gyro.getRoll()) > 4) {
                
        }
    }
}
