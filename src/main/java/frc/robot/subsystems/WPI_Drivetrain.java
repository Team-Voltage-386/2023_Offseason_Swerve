package frc.robot.subsystems;

import static frc.robot.Constants.DriveConstants.*;

import static frc.robot.Constants.AutoConstants.*;

import com.ctre.phoenix.sensors.Pigeon2;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class WPI_Drivetrain extends SubsystemBase{

    public double ypr[] = new double[3];

    public Pigeon2 gyro = new Pigeon2(kIMUid);

    public WPI_SwerveModule[] modules = {
        LeftFrontWPI, 
        RightFrontWPI,  
        LeftRearWPI, 
        RightRearWPI};

    public SwerveModulePosition[] modulePositions = {
        modules[0].getSwerveModulePosition(), 
        modules[1].getSwerveModulePosition(),
        modules[2].getSwerveModulePosition(),
        modules[3].getSwerveModulePosition()
    };

    public final SwerveDriveOdometry odometer = new SwerveDriveOdometry(kDriveKinematics, new Rotation2d(0), modulePositions);


    public WPI_Drivetrain() {
        //waits for gyro to calibrate before zeroing it.
        //does this on a different thread as to not interupt current code.
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception e) {
            }
        }).start();
    }

    //*zeros heading */
    public void zeroHeading() {
        gyro.setYaw(0);
    }
    
    //*get heading of robot in degrees*/
    public double getHeading() {
        return Math.IEEEremainder((gyro.getYaw()), 360);
    }

    public Rotation2d getHeadingRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    public Pose2d getPose() {
        return odometer.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        odometer.resetPosition(getHeadingRotation2d(), modulePositions, pose);
    }

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
                this::getPose, // Pose supplier
                kDriveKinematics, // SwerveDriveKinematics
                new PIDController(kAutoPositionPID[0], kAutoPositionPID[1], kAutoPositionPID[2]), // X controller.
                new PIDController(kAutoPositionPID[0], kAutoPositionPID[1], kAutoPositionPID[2]), // Y controller.
                new PIDController(kAutoHeadingPID[0], kAutoHeadingPID[1], kAutoHeadingPID[2]), // Rotation controller.
                this::setModuleStates, // Module states consumer
                true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
                this // Requires this drive subsystem
            )
        );
    }

    @Override
    public void periodic() {
        //update odo and swerve module data/positions
        for(int i = 0; i < modulePositions.length; i++) {
            modulePositions[i] = modules[i].getSwerveModulePosition();
        }
        odometer.update(getHeadingRotation2d(), modulePositions);
        //System.out.println("Robot Position: (" + odometer.getPoseMeters().getX() + "," + odometer.getPoseMeters().getY() + ")");

        SmartDashboard.putNumber("Robot Heading", getHeading());
        SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
        for(int i = 0; i < modules.length; i++) {
            SmartDashboard.putNumber("Wheel distance of module " + i, modules[i].getWheelPositionMeters());
            SmartDashboard.putNumber("Drive Enc Reading of module " + i, modules[i].getDriveEncPosition());
        }
        gyro.getYawPitchRoll(ypr);
    }

    public void stopModules() {
        LeftFrontWPI.stop();
        RightFrontWPI.stop();
        LeftRearWPI.stop();
        RightRearWPI.stop();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, kMaxDriveSpeed);
        LeftFrontWPI.setDesiredState(desiredStates[0]);
        RightFrontWPI.setDesiredState(desiredStates[1]);
        LeftRearWPI.setDesiredState(desiredStates[2]);
        RightRearWPI.setDesiredState(desiredStates[3]);
    }
}
