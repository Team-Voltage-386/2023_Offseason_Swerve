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
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;

public class SwerveModule {
    private static final double kWheelRadius = Units.inchesToMeters(2);
    private static final int kEncoderResolution = 4096;

    private static final double kModuleMaxAngularVelocity = Drivetrain.kMaxAngularSpeed;
    private static final double kModuleMaxAngularAcceleration = 2 * Math.PI; // radians per second squared

    private final CANSparkMax m_driveMotor;
    private final CANSparkMax m_turningMotor;
    public final double m_encoderOffset;

    private final CANCoder m_turningEncoder;

    public static final double[] kSwerveSteerPID = { 0.005, 0, 0 }; // 0.01,0.0,0.001
    public static final double[] kSwerveDrivePID = { 0, 0, 0 }; // 0.35,2,0.01

    // Gains are for example purposes only - must be determined for your own robot!
    private final PIDController m_drivePIDController = new PIDController(kSwerveDrivePID[0], kSwerveDrivePID[1],
            kSwerveDrivePID[2]);

    // Gains are for example purposes only - must be determined for your own robot!
    private final ProfiledPIDController m_turningPIDController = new ProfiledPIDController(
            kSwerveSteerPID[0], kSwerveSteerPID[1], kSwerveSteerPID[2],
            new TrapezoidProfile.Constraints(
                    kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration));

    // Gains are for example purposes only - must be determined for your own robot!
//     private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(1, 3);
//     private final SimpleMotorFeedforward m_turnFeedforward = new SimpleMotorFeedforward(1, 0.5);

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
            int driveMotorChannel,
            int turningMotorChannel,
            int turningEncoderID,
            double encoderOffset,
            boolean isReversed) {
        // The encoder: counts and measures rotation
        // The motor: does the actual work, give it power

        m_driveMotor = new CANSparkMax(driveMotorChannel, MotorType.kBrushless);
        m_turningMotor = new CANSparkMax(turningMotorChannel, MotorType.kBrushless);

        m_turningEncoder = new CANCoder(turningEncoderID);
        
        // set units of the CANCoder to radians, with velocity being radians per second
        CANCoderConfiguration config = new CANCoderConfiguration();
        config.sensorCoefficient = 2 * Math.PI / kEncoderResolution;
        config.unitString = "rad";
        config.sensorTimeBase = SensorTimeBase.PerSecond;
        m_turningEncoder.configAllSettings(config);
        m_turningEncoder.clearStickyFaults();
        m_turningEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
        m_turningEncoder.setPosition(0.0);
        m_turningEncoder.configSensorDirection(isReversed); //false is default, may need to be true for some wheels
        m_turningEncoder.configMagnetOffset(-0); //I dont know why its -0 but a big thread on delhpi was saying this was correct


        // Limit the PID Controller's input range between -pi and pi and set the input
        // to be continuous.
        m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);

        m_encoderOffset = encoderOffset;
    }

    /**
    * Returns the current state of the module.
    *
    * @return The current state of the module.
    */
    public SwerveModuleState getState() {
    return new SwerveModuleState(
    m_driveMotor.getEncoder().getVelocity(), new
    Rotation2d(getActualTurningPosition()));
    }

    /**
    * Returns the current position of the module.
    *
    * @return The current position of the module.
    */
    public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
      getActualDrivePosition(), 
      new Rotation2d(getActualTurningPosition()));
    }

    // getActualDrivePosition with math instead of the drive conversion. should output same thing if conversion is tuned. disable PosConversion in constructor.
    /**
     * Returns rotations*2*Pi*R
     * gets real position the wheel thinks it has spun.
     * 
     * @return distance wheel has gone across the floor. (Circumference*rotations)
     */
    public double getActualDrivePosition() {
      return m_driveMotor.getEncoder().getPosition()*2*Math.PI*kWheelRadius;
    }

    //GET OFFSETS IN RADS
    //MEASURING 0-2PI AND WHEN GOING PAST 2PI JUMPING BACK TO 0. WE NEED -180/180
    /**
     * Returns irl orientation of wheel, accounting for encoder offsets. 0 is when aligned with forward axis of the chasis.
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
        // Optimize the reference state to avoid spinning further than 90 degrees
        SwerveModuleState state = SwerveModuleState.optimize(desiredState,
                new Rotation2d(getActualTurningPosition()));

        // Calculate the drive output from the drive PID controller.
        final double driveOutput = m_drivePIDController.calculate(m_driveMotor.getEncoder().getVelocity(),
                state.speedMetersPerSecond);

        //final double driveFeedforward = m_driveFeedforward.calculate(state.speedMetersPerSecond);

        // Calculate the turning motor output from the turning PID controller.
        final double turnOutput = m_turningPIDController.calculate(
                getActualTurningPosition(),
                state.angle.getRadians());

        //final double turnFeedforward = m_turnFeedforward.calculate(m_turningPIDController.getSetpoint().velocity);

        m_driveMotor.setVoltage(driveOutput); // + driveFeedforward);
        m_turningMotor.setVoltage(turnOutput); // + turnFeedforward);
    }
}
