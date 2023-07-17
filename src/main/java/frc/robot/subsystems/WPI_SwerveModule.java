package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// import frc.robot.utils.PIDShufflable;
import frc.robot.Constants.DriveConstants;

public class WPI_SwerveModule extends SubsystemBase{

    public final CANSparkMax turningMotor;
    public final CANSparkMax driveMotor;
    public final CANCoder turningEncoder;
    public final PIDController turningPIDController;
    public final PIDController drivePIDController;
    public final double encoderOffs;
    public double distFromCenter;
    public SwerveModulePosition swerveModulePosition;

    public double targetSteer = 0;
    public double targetDrive = 0;

    public int driveMult = 1;

    public int swerveModuleID;
    public static int swerveModuleCount = 0;
    public final ShuffleboardTab swerveTab;
    public final GenericEntry steerMotorCurrentWidget;
    public final GenericEntry driveMotorCurrentWidget;
    public final GenericEntry driveMotorSetWidget;
    public final GenericEntry posiitonWidget;
    public final GenericEntry MotorPositionWidget;

    public WPI_SwerveModule(int STEERMOTOR, int DRIVEMOTOR, double driveConversion, double[] steerPIDValue,
            double[] drivePIDValue, int encoderID, double ENCOFFS, String SwerveModuleName) {
        
        swerveModuleID = swerveModuleCount;

        turningPIDController = new PIDController(steerPIDValue[0], steerPIDValue[1], steerPIDValue[2]);
        drivePIDController = new PIDController(drivePIDValue[0], drivePIDValue[1], drivePIDValue[2]);
        turningMotor = new CANSparkMax(STEERMOTOR, MotorType.kBrushless);
        driveMotor = new CANSparkMax(DRIVEMOTOR, MotorType.kBrushless);
        driveMotor.getEncoder().setPositionConversionFactor(driveConversion);
        driveMotor.getEncoder().setVelocityConversionFactor(driveConversion);
        turningEncoder = new CANCoder(encoderID);
        swerveModulePosition = this.getSwerveModulePosition();


        encoderOffs = ENCOFFS;

        swerveTab = Shuffleboard.getTab("SwerveModules");
        steerMotorCurrentWidget = swerveTab.add("turningMotor" + SwerveModuleName, 0).withPosition(5, swerveModuleID)
                .withSize(1, 1)
                .getEntry();
        driveMotorCurrentWidget = swerveTab.add("drive motor" + SwerveModuleName, 0).withPosition(6,
                swerveModuleID).withSize(1, 1)
                .getEntry();
        posiitonWidget = swerveTab.add("orientation" + SwerveModuleName, 0).withPosition(7,
                swerveModuleID).withSize(1, 1)
                .getEntry();
        driveMotorSetWidget = swerveTab.add("dmSET" + SwerveModuleName, 0).withPosition(8, swerveModuleID).withSize(1, 1)
                .getEntry();
        MotorPositionWidget = swerveTab.add("Module Position" + SwerveModuleName, 0).withPosition(9, swerveModuleID).withSize(1, 1)
                .getEntry();
        swerveModuleCount++;
        resetEncoders();
        // updateShufflables();
    }

    public void resetEncoders() {
        driveMotor.getEncoder().setPosition(0);
        turningEncoder.setPosition(0);
    }

    public double getTurnEncPositionDeg() {
        return turningEncoder.getAbsolutePosition() - encoderOffs;
    }

    // public double getWheelRotation() {
    //     return getTurnEncPositionDeg()/DriveConstants.kSteerMotorGearRatio;
    // }

    public double getDriveEncPosition() {
        return driveMotor.getEncoder().getPosition();
    }

    public double getTurnEncVelocity() {
        return turningEncoder.getVelocity();
    }

    public double getDriveEncVelocity() {
        return driveMotor.getEncoder().getVelocity();
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveEncVelocity(), new Rotation2d(Math.toRadians(getTurnEncPositionDeg())));
    }

    public SwerveModulePosition getSwerveModulePosition() {
        swerveModulePosition = new SwerveModulePosition(getWheelPositionMeters(), new Rotation2d(Math.toRadians(getTurnEncPositionDeg())));
        return swerveModulePosition;
    }

    public double getWheelPositionMeters() {
        //2*Pi*r (r = radius of wheels = 2 in)
        return DriveConstants.kWheelRadiusMeters*2*Math.PI*getDriveEncPosition();
    }

    public void setDesiredState(SwerveModuleState state) {
        //if let go of stick, drift for a bit
        if(Math.abs(state.speedMetersPerSecond) < 0.01) {
            stop();
            return;
        }
        state = SwerveModuleState.optimize(state, getState().angle);
        driveMotor.set(state.speedMetersPerSecond / DriveConstants.kMaxDriveSpeed);
        turningMotor.set(turningPIDController.calculate(getTurnEncPositionDeg(), state.angle.getDegrees()));
    }

    public void stop() {
        driveMotor.set(0);
        turningMotor.set(0);
    }

    @Override
    public void periodic() {
        steerMotorCurrentWidget.setDouble(getTurnEncPositionDeg());
        driveMotorCurrentWidget.setDouble(getDriveEncPosition());
    }
}
