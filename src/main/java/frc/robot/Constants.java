package frc.robot;

import edu.wpi.first.wpilibj.Joystick;

public class Constants {
    /** the indexes to address buttons on the controller */
    public static final class ControllerConstants {
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
