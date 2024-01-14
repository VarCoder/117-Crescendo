package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.misc.newConstants;
import frc.robot.subsystems.DriveSubsystem;

public class TeleopSwerve extends Command{
    private DriveSubsystem swerve;
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotSup;
    private BooleanSupplier FOCSup;

    public TeleopSwerve (DriveSubsystem swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotSup, BooleanSupplier FOCSup){
        this.swerve = swerve;
        addRequirements(swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotSup = rotSup;
        this.FOCSup = FOCSup;
    }

    @Override
    public void execute(){
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(),
                    newConstants.Swerve.stickDeadband);
            double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), newConstants.Swerve.stickDeadband);
            double rotationVal = 0.5* MathUtil.applyDeadband(rotSup.getAsDouble(), newConstants.Swerve.stickDeadband);

            /* Drive */
            swerve.drive(
                new Translation2d(translationVal, strafeVal).times(newConstants.Swerve.maxSpeed),
                rotationVal * newConstants.Swerve.maxAngularVelocity,
                !FOCSup.getAsBoolean(),
                true
            );
    }
}
