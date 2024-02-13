package org.carlmontrobotics;

public final class Constants {
    // public static final class Drivetrain {
    //     public static final double MAX_SPEED_MPS = 2;
    // }
    public static final class SHOOTER {
        public static final double kP = 0.0001;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kS = 0.29753;
        public static final double kV = 0.077913;
        public static final double kA = 0.05289;

    }
    public static final class OI {
        public static final class Driver {
            public static final int port = 0;
        }
        public static final class Manipulator {
            public static final int port = 1;
        }
    }
}
