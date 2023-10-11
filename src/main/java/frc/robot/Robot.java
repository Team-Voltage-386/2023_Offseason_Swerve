// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import Subsytems.Pneumatics;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Commands.ManipulatorCommands;
import frc.robot.Constants.Deadbands;
import frc.robot.Constants.Controller;

public class Robot extends TimedRobot {
    private final XboxController m_controller = new XboxController(Controller.kDriveController);
    private final Drivetrain m_swerve = new Drivetrain();

    private Pneumatics m_Pneumatics = new Pneumatics();
    private final ManipulatorCommands m_manipulatorCommand = new ManipulatorCommands(m_Pneumatics);

    private final SendableChooser<Integer> autoChooser = new SendableChooser<>();

    int m_autonomousCommand;

    double autoStartTime;
    double time;

    @Override
    public void robotInit()
    {
        autoChooser.addOption("Backup", 1);
        autoChooser.addOption("Score and backup", 2);
        autoChooser.addOption("Letgo", 3);
        autoChooser.addOption("Null", 0);
        Shuffleboard.getTab("Main").add("AutoRoutine", autoChooser).withSize(3, 1).withPosition(4, 2);
    }


    @Override
    public void teleopInit() {

    }

    @Override
    public void autonomousInit() {
        m_autonomousCommand = autoChooser.getSelected();
        autoStartTime = Timer.getFPGATimestamp();

    }

    // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
    private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(Controller.kRateLimitXSpeed);
    private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(Controller.kRateLimitYSpeed);
    private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(Controller.kRateLimitRot);

    @Override
    public void autonomousPeriodic() {
        if (m_autonomousCommand==1)
        {
            driveBackwardsAuto(true);
        }

        if (m_autonomousCommand==2)
        {
                driveBackwardsAndLetGoAuto(true);
        }

        if(m_autonomousCommand == 3) {
            LetGoAuto();
        }
        
        
        m_swerve.updateOdometry();
        time = Timer.getFPGATimestamp() - autoStartTime;
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
        if(time < 9.5)
            {
                System.out.println(time);
                //drive backwards at half max speed
                final var xSpeed = -m_xspeedLimiter.calculate(0.3) * Drivetrain.kMaxSpeed;

                // Get the y speed or sideways/strafe speed which should be 0
                final var ySpeed = 0;

                // dont spin
                final var rot = 0;

                m_swerve.drive(xSpeed, ySpeed, rot, fieldRelative);
            }
        else {m_swerve.drive(0, 0, 0, fieldRelative);}
    }

    boolean flag = false;
    boolean flag2 = false;
    private void driveBackwardsAndLetGoAuto(boolean fieldRelative) {
        if(time < 10.5) {
            if(!flag) {
                m_Pneumatics.disableLift();
                flag = true;
            }

            if(!flag2 && time > 1) {
                m_Pneumatics.disableCone();
                m_Pneumatics.enableCube();
                flag2 = true;
            }

            if(flag2) {
                //drive backwards at half max speed
                final var xSpeed = -m_xspeedLimiter.calculate(0.3) * Drivetrain.kMaxSpeed;

                // Get the y speed or sideways/strafe speed which should be 0
                final var ySpeed = 0;

                // dont spin
                final var rot = 0;

                m_swerve.drive(xSpeed, ySpeed, rot, fieldRelative);
            }
        }
        else {
            m_swerve.drive(0, 0, 0, fieldRelative);
        }
    }

    private void LetGoAuto() {
        if(!flag) {
            m_Pneumatics.disableLift();
            flag = true;
        }

        if(!flag2 && time > 2) {
            m_Pneumatics.disableCone();
            m_Pneumatics.enableCube();
            flag2 = true;
        }
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
