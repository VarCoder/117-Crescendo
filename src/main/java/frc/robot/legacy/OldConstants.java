package frc.robot.legacy;

import java.util.function.*;



// import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.AnalogInput;
import frc.robot.misc.SwerveModule;
import frc.robot.sensors.ThriftyEncoder;

public final class OldConstants {

    // Ports, think DIO and CAN IDs
    // Anything that is a completely arbitrary aspect
    // of how things are laid out
    public static final class Port {

        public static final class Driver {
            
            public static final int CONTROLLER = 0;

            public static final int AX_FWD = 1;
            public static final int AX_STRAFE = 0;
            public static final int AX_ROT_CW = 2;

            public static final int BTN_SUPRESS_FOC = 6;

        }

        public static final class Operator {

            public static final int BUTTONBOARD = 2;
        }

        public static final SwerveModule[] SWERVE_MODULES = {
            new SwerveModule(
                7, // 7, 6, 9, 8, 3, 2, 5, 4
                6,
                new ThriftyEncoder(new AnalogInput(2)).shiftDegs(-67), //30 deg
                "FL"
            ),
            new SwerveModule(
                5,
                4,
                new ThriftyEncoder(new AnalogInput(1)).shiftDegs(30),
                
                "FR"
            ),
            new SwerveModule(
                9,
                8,
                new ThriftyEncoder(new AnalogInput(3)).shiftDegs(-54), //dec
                "BL"
            ),
            new SwerveModule(
                3,
                2,
                new ThriftyEncoder(new AnalogInput(0)).shiftDegs(-90), // -270 good for drive -90 good for turn
                "BR"
            ),
            
        };

    }

    // Properties, think gear ratios and dimensions
    // Anything that is some measurement specific to
    // what we have created
    public static final class Prop {

        public static final class Drivetrain {

            public static final Translation2d[] MODULE_LOCATIONS = {
                new Translation2d(0.381, 0.381), //.2159
                new Translation2d(0.381, -0.381),
                new Translation2d(-0.381, 0.381),
                new Translation2d(-0.381, -0.381)
            };
            public static final double loopRateDT = 0.2; // https://www.chiefdelphi.com/t/field-relative-swervedrive-drift-even-with-simulated-perfect-modules/413892/15
            
            public static final double TURNING_ANG_VEL_MAX = Math.PI * 100;
            public static final double TURNING_ANG_ACC_MAX = Math.PI * 100;


            public static final double WHEEL_RADIUS = 0.0508;
            public static final double WHEEL_GEAR_RATIO = 6.55;

            // Meters per second to rotations per minute
            public static final UnaryOperator<Double> WHEEL_MPS_TO_RPM = mps -> (
                2 * mps * (60 /( 2 * Math.PI * WHEEL_RADIUS)) * WHEEL_GEAR_RATIO
            );

            public static final UnaryOperator<Double> WHEEL_ROT_TO_M = rot -> (
                2 * rot * Math.PI * WHEEL_RADIUS/WHEEL_GEAR_RATIO
            );

            public static final double AXIS_SPEED_MAX = 4.5;//5.0;
            public static final double AXIS_ACC_MAX = 0.075;
            public static final double AXIS_DEC_MAX = 0.5;

            public static final double ANG_VEL_MAX = Math.PI;
            public static final double ANG_ACC_MAX = 0.1;
            public static final double ANG_DEC_MAX = 0.25;

            public static final boolean GYRO_REVERSED = false;

        }

        public static final class SnakeEyesCamera {

            public static final Transform3d CAMERA_TO_ROBOT = new Transform3d(
                new Translation3d(0.1905,0.2159,0),
                new Rotation3d(0.384,0,0)
            );
            public static final Transform2d TAG_TO_POST = new Transform2d(
                new Translation2d(0,0.5715),
                new Rotation2d()
            );
        }

    

    // Component properties, think ticks per rotation
    // of an encoder or the HC-SR04's conversion factors
    // Any measurement specific to a component we are using
    // Components here don't include stuff like chassis dims
    // That *could* be arbitrary because we could mount stuff
    // differently
    public static final class Comp {

        public static final class ThriftyEnc {

            public static final double STD_READ_VOLTAGE_MAX = 4.8;

        }

        public static final class Joystick {

            public static final double DEADBAND_LINEAR = 0.1; //0.05
            public static final double DEADBAND_ANG = 0.1;

            public static final double RATE_LIMIT = 3.0;

        }

    }

    }
}