package org.carlmontrobotics;

public final class Constants {
    // public static final class Drivetrain {
    //     public static final double MAX_SPEED_MPS = 2;
    // }
    public final static double kS = 0.3796;
            public static final double kV =0.0667;
            public static final double kA = 0.005977;
            
            public static final int dsPort1 = 10;
            public static final int dsPort2 = 0;
            public static final int motorPort = 1;
    public static final class OI {
        public static final class Driver {
            public static final int port = 0;
        }
        public static final class Manipulator {
            public static final int port = 1;
        }
        public static final class Intake {
            
        }
    }
}
