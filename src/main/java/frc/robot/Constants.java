package frc.robot;

import edu.wpi.first.math.util.Units;

public class Constants {
    public static class ID {
        //Swerve motor controller IDs
        public static final int kFrontLeftTurn = 1;
        public static final int kFrontLeftDrive = 2;

        public static final int kFrontRightTurn = 3;
        public static final int kFrontRightDrive = 4;

        public static final int kBackLeftTurn = 5;
        public static final int kBackLeftDrive = 6;

        public static final int kBackRightTurn = 7;
        public static final int kBackRightDrive = 8;

        //CanCoder IDs
        public static final int kFrontLeftCANCoder = 9;
        public static final int kFrontRightCANCoder = 10;
        public static final int kBackLeftCANCoder = 11;
        public static final int kBackRightCANCoder = 12;

        //Pigeon
        public static final int kGyro = 13;
    }

    public static class Deadbands {
        public static final double kLeftJoystickDeadband = 0.2;
        public static final double kRightJoyStickDeadband = 0.3;
    }

    public static class Offsets {
        //THESE VALUES MUST BE BETWEEN -180 & 180 or -Pi & Pi
        public static final double kFrontLeftOffset = Units.degreesToRadians(175.4);
        public static final double kFrontRightOffset = Units.degreesToRadians(250.4-360);
        public static final double kBackLeftOffset = Units.degreesToRadians(309.5-360);
        public static final double kBackRightOffset = Units.degreesToRadians(340.1-360);
    }
}
