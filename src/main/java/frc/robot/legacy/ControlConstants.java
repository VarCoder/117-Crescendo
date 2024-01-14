package frc.robot.legacy;

public final class ControlConstants {
    
    public static final class Driving {

        public static final double TURNING_KP = 1.6; //1.7
        public static final double TURNING_KI = 1.8; //2.0;
        public static final double TURNING_KD = 0.0;
        public static final double TURNING_KS = 0.5;
        public static final double TURNING_KV = 0.3;

        public static final double DRIVE_KP = 0.00005;
        public static final double DRIVE_KI = 0.0;
        public static final double DRIVE_KD = 0.0;
        public static final double DRIVE_KFF = 0.001;
        public static final double DRIVE_MAX_ACCEL = 250.0;

    }
    public static final class Align {

        public static final double TURNING_KP = 1.6; //1.7
        public static final double TURNING_KI = 1.8; //2.0;
        public static final double TURNING_KD = 0.0;

        public static final double DRIVE_KP = 0.35;
        public static final double DRIVE_KI = 0.0;
        public static final double DRIVE_KD = 0.0;

    }

}
