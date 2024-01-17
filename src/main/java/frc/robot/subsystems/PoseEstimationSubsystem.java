package frc.robot.subsystems;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.misc.Constants;
import frc.robot.sensors.NavX;

public class PoseEstimationSubsystem extends SubsystemBase {
    private Field2d field2d = new Field2d();
    private SwerveDrivePoseEstimator m_swervePoseEstimator;
    DriveSubsystem m_swerve;
    VisionSubsystem m_limeLight;
    NavX m_NavX;

    public PoseEstimationSubsystem(DriveSubsystem swerve, VisionSubsystem limeLight, NavX navx) {
        // TODO: Add limelight code
        m_swerve = swerve;
        m_limeLight = limeLight;
        m_NavX = navx;

        m_NavX.zeroYaw();
        m_swervePoseEstimator = new SwerveDrivePoseEstimator(
            Constants.Swerve.swerveKinematics,
            m_NavX.getAngle(),
            m_swerve.getModulePositions(),
            new Pose2d());
        SmartDashboard.putData("Field", field2d);
    }

    public void zeroAngle() {
        m_NavX.zeroYaw();
        m_swervePoseEstimator.resetPosition(
            m_NavX.getAngle(),
            m_swerve.getModulePositions(),
            new Pose2d(getPose().getTranslation(), new Rotation2d())
        );
    }

    public Pose2d getPose() {
        return m_swervePoseEstimator.getEstimatedPosition();
    }

    public void resetOdometry(Pose2d pose) {
        m_swervePoseEstimator.resetPosition(m_NavX.getAngle(),m_swerve.getModulePositions(),pose);
    }

    @Override
    public void periodic(){
        DriverStation.refreshData();
        m_swervePoseEstimator.update(m_NavX.getAngle(),m_swerve.getModulePositions());
        field2d.setRobotPose(getPose());
        SmartDashboard.putData(field2d);
    }
}
