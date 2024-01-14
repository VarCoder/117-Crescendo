package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.legacy.OldConstants;
import frc.robot.misc.Constants;
import frc.robot.sensors.ThriftyEncoder;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkPIDController.AccelStrategy;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.*;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.trajectory.*;

public class SwerveModule {
    private final String name;
    private Rotation2d lastAngle;
    public final CANSparkMax driveMotor;
    private final CANSparkMax turningMotor;


    private final RelativeEncoder driveEncoder;
    private final ThriftyEncoder turningEncoder;
    private final SparkPIDController drivePIDController;
    private final ProfiledPIDController turningPIDController = new ProfiledPIDController(
        Constants.Swerve.angleKP,
        Constants.Swerve.angleKI,
        Constants.Swerve.angleKD,
        new TrapezoidProfile.Constraints(
                Constants.Swerve.maxAngularVelocity,
                Constants.Swerve.maxAngularAccel)
    );

    private final SimpleMotorFeedforward turnFeedforward = new SimpleMotorFeedforward(
        Constants.Swerve.angleKS,
        Constants.Swerve.angleKV
    );

    private final SimpleMotorFeedforward driveFeedForward = new SimpleMotorFeedforward(
        Constants.Swerve.driveKS,
        Constants.Swerve.driveKV,
        Constants.Swerve.driveKA
    );
    public SwerveModule(int driveMotorID, int turningMotorID, ThriftyEncoder thriftyEncoder, String name) {
        this.name = name;

        /* Configure Driving Motor, Encoder, and PIDController */
        driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
        driveEncoder = driveMotor.getEncoder();
        drivePIDController = driveMotor.getPIDController();
        configDriveMotor();

        /* Configure Turning Motor, Encoder, and PIDController */
        turningMotor = new CANSparkMax(turningMotorID, MotorType.kBrushless);
        turningEncoder = thriftyEncoder;
        configTurnMotor();

        lastAngle = getState().angle;
    }

    private void configDriveMotor(){

        driveMotor.restoreFactoryDefaults();
        driveMotor.setSmartCurrentLimit(Constants.Swerve.driveContinuousCurrentLimit);
        driveMotor.setInverted(Constants.Swerve.driveInvert);
        driveMotor.setIdleMode(Constants.Swerve.driveNeutralMode);

        /* Setup Driving PID */
        drivePIDController.setP(Constants.Swerve.driveKP, 0);
        drivePIDController.setI(Constants.Swerve.driveKI, 0);
        drivePIDController.setD(Constants.Swerve.driveKD, 0);
        drivePIDController.setOutputRange(-1, 1);
        drivePIDController.setSmartMotionMaxVelocity(10000, 0);
        drivePIDController.setFF(Constants.Swerve.driveKFF, 0);
        drivePIDController.setSmartMotionAccelStrategy(AccelStrategy.kSCurve, 0);
        drivePIDController.setSmartMotionMaxAccel(Constants.Swerve.maxAccel, 0);
        driveMotor.burnFlash();
        driveEncoder.setPosition(0.0);
    }
    private void configTurnMotor(){

        turningMotor.restoreFactoryDefaults();
        turningMotor.setSmartCurrentLimit(Constants.Swerve.angleContinuousCurrentLimit);
        turningMotor.setInverted(Constants.Swerve.angleInvert);

        turningMotor.setIdleMode(Constants.Swerve.angleNeutralMode);
        /*  Limit the PID Controller's input range between -pi and pi and set the input
        to be continuous. */
        turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
        turningMotor.burnFlash();

    }

    public void setModuleIdleMode(IdleMode driveIdleMode, IdleMode turnIdleMode) {
        turningMotor.setIdleMode(turnIdleMode);
        driveMotor.setIdleMode(driveIdleMode);
    }
    public void setDesiredState(SwerveModuleState desiredState, boolean openLoop) {

        /* Optimize the reference state to avoid spinning further than 90 degrees */
        desiredState = SwerveModuleState.optimize(desiredState, turningEncoder.get());
        setAngle(desiredState);
        setSpeed(desiredState, openLoop);
        

        SmartDashboard.putNumber("Drive " + name, driveEncoder.getPosition());
        SmartDashboard.putNumber("Turn " + name,turningEncoder.get().getDegrees());
        SmartDashboard.putNumber("State " + name, desiredState.angle.getDegrees());
        SmartDashboard.putNumber("Speed " + name, desiredState.speedMetersPerSecond);

        
    }

    private void setAngle(SwerveModuleState desiredState) {
        // Prevent rotating module if speed is less then 1%. Prevents jittering.
        Rotation2d angle =
            (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.maxSpeed * 0.01))
                ? lastAngle
                : desiredState.angle;
        
        final double turnOutput = turningPIDController.calculate(angle.getRadians(),
                desiredState.angle.getRadians());
        final double turnFeedforwardOut = turnFeedforward.calculate(turningPIDController.getSetpoint().velocity);
        turningMotor.setVoltage(turnOutput + turnFeedforwardOut);
        lastAngle = angle;
    }
    private void setSpeed(SwerveModuleState desiredState, boolean openLoop){
        if (openLoop){
            double output = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
            driveMotor.set(output);
        } else{
            drivePIDController.setReference(
                desiredState.speedMetersPerSecond,
                ControlType.kVelocity, // units for setpoint (rpm)
                0,// units for feedforward,
                driveFeedForward.calculate(desiredState.speedMetersPerSecond)
            );
        }
    }
    public void reset() {
        turningEncoder.reset();
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            OldConstants.Prop.Drivetrain.WHEEL_ROT_TO_M.apply(driveEncoder.getPosition()), turningEncoder.get()
        );
    }
    public SwerveModuleState getState() {
        
        return new SwerveModuleState(driveEncoder.getVelocity(), turningEncoder.get());
    }
}
