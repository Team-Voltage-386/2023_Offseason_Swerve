package frc.robot.Commands;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Drivetrain;

public class AutoPathCommands extends CommandBase{
    public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath) {
        Drivetrain dt = new Drivetrain();
        return new SequentialCommandGroup(
            new InstantCommand(() -> {
            // Reset odometry for the first path you run during auto
                if(isFirstPath){
                    dt.resetOdometry(traj.getInitialHolonomicPose());
                }
            }),
            new PPSwerveControllerCommand(
                traj, 
                () -> dt.m_odometry.getPoseMeters(), // Pose supplier
                dt.m_kinematics, // SwerveDriveKinematics
                new PIDController(0.5, 0, 0), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
                new PIDController(0.5, 0, 0), // Y controller (usually the same values as X controller)
                new PIDController(0.5, 0, 0), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
                (SwerveModuleState[] states) -> { // Consumes the module states to set the modules moving in the directions we want
                    dt.m_chassisSpeeds = dt.m_kinematics.toChassisSpeeds(states);
                    dt.setModuleStates(states);
                }, // Module states consumer
                    true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
                    dt // Requires this drive subsystem
                )
        );
    }
}
