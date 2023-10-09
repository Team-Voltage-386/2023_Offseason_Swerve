// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModule {
    private static final double kWheelRadius = Units.inchesToMeters(2);
    private static final int kEncoderResolution = 4096;

    private static final double kModuleMaxAngularVelocity = Drivetrain.kMaxAngularSpeed;
    private static final double kModuleMaxAngularAcceleration = 2 * Math.PI; // radians per second squared

    /**
     * The drive motor is responsible for the actual power across the ground e.g. to
     * make it move forward/backward
     */
    private final CANSparkMax m_driveMotor;

    /**
     * The turning/steering motor is responsible for the orientation of the wheel
     * e.g. to make it turn
     */
    private final CANSparkMax m_turningMotor;

    /**
     * Each turning motor has an encoder. Each motor was mounted by people at
     * seemingly random offsets. We have to accomadate for these
     * offsets so that calls to getAbsolutePosition all return the same value
     * regardless of motor i.e. make them consider the same reference point as "0"
     */
    public final double m_encoderOffset;

    /**
     * This does not control the motor. This just records the orientatation of the
     * module.
     * IMPORTANT NOTE: Ensure that the encoder and motor both agree which
     * direction is positive.
     * We faced an issue where the motor oscillated inexplicably and we traced it
     * back to the motor needing to be inverted to match with the encoder.
     */
    private final CANCoder m_turningEncoder;

    /**
     * Identification for what motor this is
     */
    private final String m_swerveModuleName;

    public static final double[] kSwerveSteerPID = { 6.0, 0, 0 };
    public static final double[] kSwerveDrivePID = { 0, 0, 0 };

    private final PIDController m_drivePIDController = new PIDController(kSwerveDrivePID[0], kSwerveDrivePID[1],
            kSwerveDrivePID[2]);

    private final ProfiledPIDController m_turningPIDController = new ProfiledPIDController(
            kSwerveSteerPID[0], kSwerveSteerPID[1], kSwerveSteerPID[2],
            new TrapezoidProfile.Constraints(
                    kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration));

    // Example code came with these feed forward pieces which we haven't yet added
    // as they are meant for tuning and are not required
    // We leave them here in case you'd like to reference them
    // private final SimpleMotorFeedforward m_driveFeedforward = new
    // SimpleMotorFeedforward(1, 3);
    // private final SimpleMotorFeedforward m_turnFeedforward = new
    // SimpleMotorFeedforward(1, 0.5);

    /**
     * Constructs a SwerveModule with a drive motor, turning motor, drive encoder
     * and turning encoder.
     *
     * @param driveMotorChannel      PWM output for the drive motor.
     * @param turningMotorChannel    PWM output for the turning motor.
     * @param driveEncoderChannelA   DIO input for the drive encoder channel A
     * @param driveEncoderChannelB   DIO input for the drive encoder channel B
     * @param turningEncoderChannelA DIO input for the turning encoder channel A
     * @param turningEncoderChannelB DIO input for the turning encoder channel B
     */
    public SwerveModule(
            String swerveModuleID,
            int driveMotorChannel,
            int turningMotorChannel,
            int turningEncoderID,
            double encoderOffset) {

        m_swerveModuleName = swerveModuleID;

        /*
         * Set up the drive motor
         */
        m_driveMotor = new CANSparkMax(driveMotorChannel, MotorType.kBrushless);

        /*
         * Set up the turning motor. We had to invert the turning motor so it agreed
         * with the turning encoder which direction was positive
         */
        m_turningMotor = new CANSparkMax(turningMotorChannel, MotorType.kBrushless);
        m_turningMotor.setInverted(true);

        /*
         * Set up and configure the turning encoder. Additional config is required to
         * ensure that units are correct
         */
        m_turningEncoder = new CANCoder(turningEncoderID);

        // Set units of the CANCoder to radians, with velocity being radians per second
        CANCoderConfiguration config = new CANCoderConfiguration();
        config.sensorCoefficient = 2 * Math.PI / kEncoderResolution;
        config.unitString = "rad";
        config.sensorTimeBase = SensorTimeBase.PerSecond;
        m_turningEncoder.configAllSettings(config);
        m_turningEncoder.clearStickyFaults();
        m_turningEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);

        // Limit the PID Controller's input range between -pi and pi and set the input
        // to be continuous.
        m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);

        m_encoderOffset = encoderOffset;
    }

    /**
     * Returns the current Swerve Module state. This includes info about the module
     * wheel's speed per second and the angle of the module. Units are based on the
     * configuration of the motors and encoders
     *
     * @return The current state of the module.
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(
                m_driveMotor.getEncoder().getVelocity(), new Rotation2d(getActualTurningPosition()));
    }

    /**
     * Returns the current position of the module. This includes info about the
     * distance measured by the wheel and the angle of the module.
     *
     * @return The current position of the module.
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                getActualDrivePosition(),
                new Rotation2d(getActualTurningPosition()));
    }

    // getActualDrivePosition with math instead of the drive conversion. should
    // output same thing if conversion is tuned. disable PosConversion in
    // constructor.
    /**
     * Returns the distance the wheel thinks it has gone across the floor. We use
     * math here instead of built-in drive conversion for ease of use
     * 
     * @return distance wheel has gone across the floor. (Circumference*rotations)
     */
    public double getActualDrivePosition() {
        return m_driveMotor.getEncoder().getPosition() * 2 * Math.PI * kWheelRadius;
    }

    /**
     * Returns orientation of wheel, accounting for encoder offsets. 0 is when
     * aligned with forward axis of the chasis.
     * 
     * @return real orientation of wheel.
     */
    public double getActualTurningPosition() {
        return m_turningEncoder.getAbsolutePosition() - m_encoderOffset;
    }

    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        // Optimize the reference state to avoid spinning further than 90 degrees or
        // PI/2 radians
        SwerveModuleState state = SwerveModuleState.optimize(desiredState,
                new Rotation2d(getActualTurningPosition()));

        // Calculate the drive output from the drive PID controller.
        final double driveOutput = m_drivePIDController.calculate(m_driveMotor.getEncoder().getVelocity(),
                state.speedMetersPerSecond);

        // Calculate the turning motor output from the turning PID controller.
        final double turnOutput = m_turningPIDController.calculate(
                getActualTurningPosition(),
                state.angle.getRadians());

        // Left in from the example code we adapted, this is not required for actual use
        //but is left in case you want to try using it
        // final double driveFeedforward =
        // m_driveFeedforward.calculate(state.speedMetersPerSecond);

        // Left in from the example code we adapted, this is not required for actual use
        // but is left in case you want to try using it
        // final double turnFeedforward =
        // m_turnFeedforward.calculate(m_turningPIDController.getSetpoint().velocity);

        m_driveMotor.setVoltage(driveOutput); // + driveFeedforward);
        m_turningMotor.setVoltage(turnOutput); // + turnFeedforward);

        SmartDashboard.putNumber(m_swerveModuleName + " Actual Turning Position", getActualTurningPosition());
        SmartDashboard.putNumber(m_swerveModuleName + " Target Turning Position", state.angle.getRadians());
        SmartDashboard.putNumber(m_swerveModuleName + " Diff Turning Position",
                getActualTurningPosition() - state.angle.getRadians());
    }

    /**
     * Put info on smart dashboard
     */
    public void print() {
        SmartDashboard.putNumber(m_swerveModuleName + " Absolute Position", m_turningEncoder.getAbsolutePosition());
    }
}
