// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
// import edu.wpi.first.wpilibj.DoubleSolenoid;
// import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
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

    @Override
    public void autonomousPeriodic() {
        while(Timer.getFPGATimestamp() < 1);
        driveBackwardsAuto(true);
        
        m_swerve.updateOdometry();
    }

    @Override
    public void teleopPeriodic() {
        driveWithJoystick(true);
        m_swerve.updateOdometry();
    }

    @Override
    public void disabledPeriodic() {
        // Only needed when measuring and configuring the encoder offsets. Can comment
        // out when not used
        m_swerve.print();
    }

    private void driveBackwardsAuto(boolean fieldRelative) {
        //drive backwards at half max speed
        final var xSpeed = -m_xspeedLimiter.calculate(0.5) * Drivetrain.kMaxSpeed;

        // Get the y speed or sideways/strafe speed which should be 0
        final var ySpeed = 0;

        // dont spin
        final var rot = 0;

        m_swerve.drive(xSpeed, ySpeed, rot, fieldRelative);
    }

    private void driveWithJoystick(boolean fieldRelative) {
        // Get the x speed. We are inverting this because Xbox controllers return
        // negative values when we push forward.
        final var xSpeed = -m_xspeedLimiter
                .calculate(MathUtil.applyDeadband(m_controller.getLeftY(), Deadbands.kLeftJoystickDeadband))
                * Drivetrain.kMaxSpeed;

        // Get the y speed or sideways/strafe speed. We are inverting this because
        // we want a positive value when we pull to the left. Xbox controllers
        // return positive values when you pull to the right by default.
        final var ySpeed = -m_yspeedLimiter
                .calculate(MathUtil.applyDeadband(m_controller.getLeftX(), Deadbands.kLeftJoystickDeadband))
                * Drivetrain.kMaxSpeed;

        // Get the rate of angular rotation. We are inverting this because we want a
        // positive value when we pull to the left (remember, CCW is positive in
        // mathematics). Xbox controllers return positive values when you pull to
        // the right by default.
        final var rot = -m_rotLimiter
                .calculate(MathUtil.applyDeadband(m_controller.getRightX(), Deadbands.kRightJoyStickDeadband))
                * Drivetrain.kMaxAngularSpeed;

        m_swerve.drive(xSpeed, ySpeed, rot, fieldRelative);

        if (m_controller.getRightBumperPressed()) {
            m_swerve.resetGyro();
        }
    }
}
