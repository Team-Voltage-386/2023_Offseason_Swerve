// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Deadbands;
import frc.robot.Constants.Controller;;

public class Robot extends TimedRobot {
    // private final DoubleSolenoid m_doubleSolenoid = new DoubleSolenoid(0,
    // PneumaticsModuleType.CTREPCM, 0, 1);

    private final XboxController m_controller = new XboxController(Controller.kDriveController);
    private final Drivetrain m_swerve = new Drivetrain();

    // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
    private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(Controller.kRateLimitXSpeed);
    private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(Controller.kRateLimitYSpeed);
    private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(Controller.kRateLimitRot);

    Command path = m_swerve.followTrajectoryCommand(PathPlanner.loadPath("New Path", 0.4, 1.0, false), true);

    @Override
    public void robotInit() {
        
    }

    @Override
    public void autonomousInit() {
        path.schedule();
    }

    @Override
    public void autonomousPeriodic() {
        //driveWithJoystick(false);
        m_swerve.updateOdometry();
        path.execute();
    }

    @Override
    public void teleopInit() {
        path.cancel();
    }

    @Override
    public void teleopPeriodic() {
        driveWithJoystick(true);
    }

    @Override
    public void disabledPeriodic() {
        // Only needed when measuring and configuring the encoder offsets. Can comment
        // out when not used
        m_swerve.print();
    }

    private void driveWithJoystick(boolean fieldRelative) {
        m_swerve.print();
        // Get the x speed. We are inverting this because Xbox controllers return
        // negative values when we push forward.
        final var xSpeed = -m_xspeedLimiter
                .calculate(MathUtil.applyDeadband(m_controller.getLeftY(), Deadbands.kLeftJoystickDeadband))
                * Constants.Controller.kMaxNecessarySpeed;

        // Get the y speed or sideways/strafe speed. We are inverting this because
        // we want a positive value when we pull to the left. Xbox controllers
        // return positive values when you pull to the right by default.
        final var ySpeed = -m_yspeedLimiter
                .calculate(MathUtil.applyDeadband(m_controller.getLeftX(), Deadbands.kLeftJoystickDeadband))
                * Constants.Controller.kMaxNecessarySpeed;

        // Get the rate of angular rotation. We are inverting this because we want a
        // positive value when we pull to the left (remember, CCW is positive in
        // mathematics). Xbox controllers return positive values when you pull to
        // the right by default.
        final var rot = -m_rotLimiter
                .calculate(MathUtil.applyDeadband(m_controller.getRightX(), Deadbands.kRightJoyStickDeadband))
                * Drivetrain.kMaxAngularSpeed;

        m_swerve.drive(xSpeed, ySpeed, rot, fieldRelative);
        SmartDashboard.putNumber("Input rot speed", MathUtil.applyDeadband(m_controller.getRightX(), Deadbands.kRightJoyStickDeadband));
        

        if (m_controller.getRightBumperPressed()) {
            m_swerve.resetOdo(new Pose2d());
        }
    }
}
