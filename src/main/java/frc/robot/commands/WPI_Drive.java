package frc.robot.commands;

import java.util.function.Supplier;
import static frc.robot.Constants.ControllerConstants.*;
import static frc.robot.Constants.DriveConstants.*;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.WPI_Drivetrain;

public class WPI_Drive extends CommandBase{
    
    private final WPI_Drivetrain dt;
    private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction;
    private final Supplier<Boolean> fieldOrientedFunction;
    private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;

    public WPI_Drive(WPI_Drivetrain dt,
            Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction, Supplier<Double> turningSpdFunction,
            Supplier<Boolean> fieldOrientedFunction) {
        this.dt = dt;
        this.xSpdFunction = xSpdFunction;
        this.ySpdFunction = ySpdFunction;
        this.turningSpdFunction = turningSpdFunction;
        this.fieldOrientedFunction = fieldOrientedFunction;
        this.xLimiter = new SlewRateLimiter(kMaxDriveAccel);
        this.yLimiter = new SlewRateLimiter(kMaxDriveAccel);
        this.turningLimiter = new SlewRateLimiter(kMaxRotAccel);
        addRequirements(dt);
    }

    @Override
    public void initialize() {
        
    }
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        //read joysticks
        double xSpeed = xSpdFunction.get();
        double ySpeed = ySpdFunction.get();
        double turningSpeed = turningSpdFunction.get();

        //deadband
        xSpeed = Math.abs(xSpeed) > kJoyStickDeadzone ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > kJoyStickDeadzone ? ySpeed : 0.0;
        turningSpeed = Math.abs(turningSpeed) > kJoyStickDeadzone ? turningSpeed : 0.0;

        //rate limiter so stuff isnt jerky
        xSpeed = xLimiter.calculate(xSpeed)*kMaxDriveSpeed/4;
        ySpeed = yLimiter.calculate(ySpeed)*kMaxDriveSpeed/4;
        turningSpeed = turningLimiter.calculate(turningSpeed)*kMaxRotSpeed/4;

        //Construct chassis speeds
        ChassisSpeeds chassisSpeeds;
        if(fieldOrientedFunction.get()) {
            //field oriented
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turningSpeed, dt.getHeadingRotation2d());
        } else {
            //robo oriented
            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
        }

        SwerveModuleState[] moduleStates = kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

        dt.setModuleStates(moduleStates);
    }
  
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        dt.stopModules();
    }
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return false;
    }
}
