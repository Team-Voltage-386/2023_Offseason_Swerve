package frc.robot;

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
        public static final int kYLeft = -1;
        public static final int kYRight = 1;
    };

    public static class Controller {
        public static final int kDriveController = 1;

        /**
         * Rate limiters make joystick inputs more gentle; 1/3 sec from 0 to 1.
         */
        public static final double kRateLimitXSpeed = 3.0;
        public static final double kRateLimitYSpeed = 3.0;
        public static final double kRateLimitRot = 3.0;
    }

    public static class Offsets {
        // Ensure that the encoder offsets are between -Pi & Pi

        /**
         * Encoder offsets
         */
        public static final double kFrontLeftOffset = 3.1035;
        public static final double kFrontRightOffset = -1.8959;
        public static final double kBackLeftOffset = -0.8375;
        public static final double kBackRightOffset = -0.3880;
    }
}
