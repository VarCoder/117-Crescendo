package frc.robot.subsystems;


import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.legacy.Constants;
import frc.robot.misc.newConstants;
import frc.robot.misc.newConstants.Swerve.*;
import frc.robot.sensors.NavX;

public class DriveSubsystem extends SubsystemBase {
	// Robot swerve modules
	private final SwerveModule[] swerveModules = Constants.Port.SWERVE_MODULES;


	public NavX navx = new NavX();
	public Field2d field2d = new Field2d();
	public IdleMode adjustableDriveNeutralMode;
	public IdleMode adjustableTurnNeutralMode;
	// Odometry class for tracking robot pose
	public SwerveDrivePoseEstimator swerveOdometry;
	public boolean openLoop;
	/** Creates a new DriveSubsystem. */
	public DriveSubsystem() {
		navx.reset();
		swerveOdometry = new SwerveDrivePoseEstimator(
			newConstants.Swerve.swerveKinematics,
			navx.getAngle(),
			new SwerveModulePosition[] {
					Mod0.module.getPosition(),
					Mod1.module.getPosition(),
					Mod2.module.getPosition(),
					Mod3.module.getPosition()
			},
			new Pose2d()
		);
		SmartDashboard.putData("Field", field2d);
		Timer.delay(1);
		resetEncoders();
	}

	@Override
	public void periodic() {
		/* Update Odometry */
		swerveOdometry.update(navx.getAngle(), getModulePositions());
		

		/*  */
		Mod0.module.setModuleIdleMode(adjustableDriveNeutralMode,adjustableTurnNeutralMode);
		Mod1.module.setModuleIdleMode(adjustableDriveNeutralMode,adjustableTurnNeutralMode);
		Mod2.module.setModuleIdleMode(adjustableDriveNeutralMode,adjustableTurnNeutralMode);
		Mod3.module.setModuleIdleMode(adjustableDriveNeutralMode,adjustableTurnNeutralMode);
		field2d.setRobotPose(swerveOdometry.getEstimatedPosition());
	}

	// // Update the odometry in the periodic block
	/**
	 * Method to drive the robot using joystick info.
	 *
	 * @param xSpeed        Speed of the robot in the x direction (forward).
	 * @param ySpeed        Speed of the robot in the y direction (sideways).
	 * @param rot           Angular rate of the robot.
	 * @param fieldRelative Whether the provided x and y speeds are relative to the
	 *                      field.
	 */
	public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean openLoop) {
		this.openLoop = openLoop;
		var swerveModuleStates = newConstants.Swerve.swerveKinematics.toSwerveModuleStates(
				fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
					translation.getX(),
					translation.getY(),
					rotation,
					navx.getAngle()
				) : new ChassisSpeeds(
					translation.getX(),
					translation.getY(),
					rotation
					)
				);
		SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates,newConstants.Swerve.maxSpeed);
		Mod0.module.setDesiredState(swerveModuleStates[0], openLoop);
		Mod1.module.setDesiredState(swerveModuleStates[1], openLoop);
		Mod2.module.setDesiredState(swerveModuleStates[2], openLoop);
		Mod3.module.setDesiredState(swerveModuleStates[3], openLoop);

	}
	/**
	 * Sets the swerve ModuleStates.
	 *
	 * @param desiredStates The desired SwerveModule states.
	 */
	public void setModuleStates(SwerveModuleState[] desiredStates) {
		SwerveDriveKinematics.desaturateWheelSpeeds(
				desiredStates, Constants.Prop.Drivetrain.AXIS_SPEED_MAX);
		Mod0.module.setDesiredState(desiredStates[0], false);
		Mod1.module.setDesiredState(desiredStates[1], false);
		Mod2.module.setDesiredState(desiredStates[2], false);
		Mod3.module.setDesiredState(desiredStates[3], false);
	}

	/** Resets the drive encoders to currently read a position of 0. */
	public void resetEncoders() {
		Mod0.module.reset();
		Mod1.module.reset();
		Mod2.module.reset();
		Mod3.module.reset();
	}

	public void resetOdometry(Pose2d pose){
		swerveOdometry.resetPosition(navx.getAngle(),getModulePositions(),pose);
	}

	/** Zeroes the heading of the robot. */
	public void zeroHeading() {
		navx.zeroYaw();
	}

	/**
	 * Returns the heading of the robot.
	 *
	 * @return the robot's heading in degrees, from -180 to 180
	 */
	public double getHeading() {
		return navx.getAngle().getDegrees();
	}

	public SwerveModulePosition[] getModulePositions() {
		return new SwerveModulePosition[] {
				swerveModules[0].getPosition(),
				swerveModules[1].getPosition(),
				swerveModules[2].getPosition(),
				swerveModules[3].getPosition()
		};
	}	
}
