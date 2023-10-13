package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;

public class Constants {
    public static class ID {
        // Swerve motor controller IDs
        public static final int kFrontLeftTurn = 1;
        public static final int kFrontLeftDrive = 2;

        public static final int kFrontRightTurn = 3;
        public static final int kFrontRightDrive = 4;

        public static final int kBackRightTurn = 5;
        public static final int kBackRightDrive = 6;

        public static final int kBackLeftTurn = 7;
        public static final int kBackLeftDrive = 8;

        // CanCoder IDs
        public static final int kFrontLeftCANCoder = 9;
        public static final int kFrontRightCANCoder = 10;
        public static final int kBackRightCANCoder = 11;
        public static final int kBackLeftCANCoder = 12;

        // Pigeon
        public static final int kGyro = 13;
    }

    public static class Deadbands {
        public static final double kLeftJoystickDeadband = 0.2;
        public static final double kRightJoyStickDeadband = 0.3;
    }

    public static class DriveTrain {
        public static final double kDistanceMiddleToFrontMotor = 0.365125;
        public static final double kDistanceMiddleToSideMotor = 0.263525;

        public static final int kXForward = 1;
        public static final int kXBackward = -1;
        public static final int kYLeft = 1;
        public static final int kYRight = -1;
    };

    public static class Controller {
        public static final int kDriveController = 0;

        /**
         * Rate limiters make joystick inputs more gentle; 1/3 sec from 0 to 1.
         */
        public static final double kRateLimitXSpeed = 100.0;
        public static final double kRateLimitYSpeed = 100.0;
        public static final double kRateLimitRot = 100.0;
    }

    public static class Offsets {
        // Ensure that the encoder offsets are between -Pi & Pi

        /**
         * Encoder offsets
         */
        public static final double kFrontLeftOffset = 3.1032;
        public static final double kFrontRightOffset = -1.8945;
        public static final double kBackLeftOffset = -0.8606;
        public static final double kBackRightOffset = -0.3053;
    }

    /** the indexes to address buttons on the controller */
    public static final class ControllerConstants {
        public static final double kDeadband = .1;
        // public static final Joystick kDriver = new Joystick(0);
        public static final Joystick kManipulator = new Joystick(1);

        public static final int kLeftVertical = 1;
        public static final int kRightVertical = 5;
        public static final int kLeftHorizontal = 0;
        public static final int kRightHorizontal = 4;
        public static final int kLeftTrigger = 2;
        public static final int kRightTrigger = 3;

        public static final int kA = 1;
        public static final int kB = 2;
        public static final int kX = 3;
        public static final int kY = 4;
        public static final int kLeftBumper = 5;
        public static final int kRightBumper = 6;
        public static final int kLeftOptions = 7;
        public static final int kRightOptions = 8;
        public static final int kLeftJoystickPressed = 9;
        public static final int kRightJoystickPressed = 10;
    }
}
