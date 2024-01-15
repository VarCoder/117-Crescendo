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
    public NavX navx = new NavX();
    public Field2d field2d = new Field2d();
    static SwerveDrivePoseEstimator swervePoseEst;
    DriveSubsystem m_swerve;
    VisionSubsystem m_limeLight;
    NavX m_NavX;

    public PoseEstimationSubsystem(DriveSubsystem swerve, VisionSubsystem limeLight, NavX navx) {
        // TODO: Add limelight code
        m_swerve = swerve;
        m_limeLight = limeLight;
        m_NavX = navx;

        m_NavX.zeroYaw();
        swervePoseEst = new SwerveDrivePoseEstimator(
            Constants.Swerve.swerveKinematics,
            m_NavX.getAngle(),
            m_swerve.getModulePositions(),
        new Pose2d());
        SmartDashboard.putData("Field", field2d);
    }

    public void zeroAngle() {
        m_NavX.zeroYaw();
        swervePoseEst.resetPosition(
            m_NavX.getAngle(),
            m_swerve.getModulePositions(),
            new Pose2d(getPose().getTranslation(), new Rotation2d())
        );
    }

    public Pose2d getPose() {
        return swervePoseEst.getEstimatedPosition();
    }

    public void resetOdometry(Pose2d pose) {
        swervePoseEst.resetPosition(m_NavX.getAngle(),m_swerve.getModulePositions(),pose);
    }

    @Override
    public void periodic(){
        DriverStation.refreshData();
        swervePoseEst.update(m_NavX.getAngle(),m_swerve.getModulePositions());
        field2d.setRobotPose(swervePoseEst.getEstimatedPosition());
        SmartDashboard.putData(field2d);
    }
}
