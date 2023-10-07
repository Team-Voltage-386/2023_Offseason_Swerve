package frc.robot;

import edu.wpi.first.math.util.Units;

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

    public static class Offsets {
        // Ensures that the offsets are between -Pi & Pi
        public static final double kFrontLeftOffset = 3.1035;
        public static final double kFrontRightOffset = -1.8959;
        public static final double kBackLeftOffset = -0.8375;
        public static final double kBackRightOffset = -0.3880;
    }
}
