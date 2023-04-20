// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.WPI_Drive;
import frc.robot.subsystems.WPI_Drivetrain;
import frc.robot.utils.PathGroupUtils;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static frc.robot.Constants.ControllerConstants.*;
import static frc.robot.Constants.DriveConstants.*;
import static frc.robot.Constants.AutoConstants.*;
import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    public final WPI_Drivetrain m_driveTrain = new WPI_Drivetrain();
    public final PathGroupUtils PGU = new PathGroupUtils();
    private final AutoRoutines autos = this.new AutoRoutines();

    private final SendableChooser<Command> autoChooser = new SendableChooser<>();

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        m_driveTrain.setDefaultCommand(new WPI_Drive(
                m_driveTrain,
                () -> -kDriver.getRawAxis(kLeftVertical),
                () -> kDriver.getRawAxis(kLeftHorizontal),
                () -> kDriver.getRawAxis(kRightHorizontal),
                () -> !(kDriver.getRawAxis(kRightTrigger) < 0.5))
        );

        configureBindings();
    }

        

        //Create the AutoBuilder. This only needs to be created once when robot code starts, not every time you want to create an auto command. A good place to put this is in RobotContainer along with your subsystems.
        SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
            m_driveTrain::getPose, // Pose2d supplier
            m_driveTrain::resetOdometry, // Pose2d consumer, used to reset odometry at the beginning of auto
            kDriveKinematics, // SwerveDriveKinematics
            new PIDConstants(5.0, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
            new PIDConstants(0.5, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
            m_driveTrain::setModuleStates, // Module states consumer used to output to the drive subsystem
            kEventMap,
            true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
            m_driveTrain // The drive subsystem. Used to properly set the requirements of path following commands
        );

        double[] maxVelo = {4, 4, 4, 4};
        double[] maxAccel = {3, 1, 3, 2};
        //Command pg1 = autoBuilder.fullAuto(PGU.generatePathGroup("Path Group 1", maxVelo, maxAccel));

        PathPlannerTrajectory TestPath = PathPlanner.loadPath("Test Path 1", new PathConstraints(4, 3));

        public final class AutoRoutines {
            // public final Command TestPP = new SequentialCommandGroup(
            //     m_driveTrain.followTrajectoryCommand(TestPath, true)
            // );
        }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be
     * created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
     * an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
     * {@link
     * CommandXboxController
     * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or
     * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings() {
        new JoystickButton(kDriver, kB).whenPressed(() -> m_driveTrain.zeroHeading());
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
