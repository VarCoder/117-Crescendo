package frc.robot.sensors;

import java.util.function.*;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;

public class ThriftyEncoder implements Supplier<Rotation2d> {
    private final AnalogInput encoder;
    private boolean inverted = false;
    public Rotation2d offset = new Rotation2d(0.0);

    public ThriftyEncoder(AnalogInput encoder) {
        this.encoder = encoder;
    }

    public void configure(boolean inverted){ //TODO: Figure out if any of the encoders are inverted
        this.inverted = inverted;
    }
    // Does not include offset
    private double getRawPositionHelper() {
        return (inverted ? -1.0 : 1.0) * (encoder.getAverageVoltage() / RobotController.getVoltage5V()) * 360;
        // return new Rotation2d(
        //     (encoder.getVoltage() * 2 * Math.PI) / read_voltage_max
        // );
    }

    public Rotation2d getRawPosition(){
        return new Rotation2d(Units.degreesToRadians(getRawPositionHelper()));
    }
    // Does include offset
    public Rotation2d get() {
        return getRawPosition().plus(offset);
    }

    public ThriftyEncoder reset() {
        offset = getRawPosition();
        return this;
    }

    // Update the offset (radians)
    public ThriftyEncoder shiftRads(double radians) {
        offset = offset.plus(new Rotation2d(radians));
        return this;
    }

    // Update the offset (degrees)
    public ThriftyEncoder shiftDegs(double degrees) {
        offset = offset.plus(Rotation2d.fromDegrees(degrees));
        return this;
    }
}
