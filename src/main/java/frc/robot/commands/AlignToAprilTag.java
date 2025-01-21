package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.libs.LimelightHelpers;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;

public class AlignToAprilTag extends Command {

    ProfiledPIDController rotationController = new ProfiledPIDController(6, 0, 0, new Constraints(Constants.maxAngularVelocity, 4));
    ProfiledPIDController strafeController = new ProfiledPIDController(3, 0, 0, new Constraints(Constants.maxSpeed, 2));
    ProfiledPIDController zController = new ProfiledPIDController(3, 0, 0, new Constraints(Constants.maxSpeed, 2));

    PoseEstimatorSubsystem m_poseEstimaorSubsystem;
    DrivetrainSubsystem m_drivetrainSubsystem;

    double rotationTolerance = Math.toRadians(0.5);
    double strafeTolerance = 0.01;
    double zTolerance = 0.01;

    double currAngle;
    double currStrafeError;
    double currZError;

    double rotSpeed;
    double strafeSpeed;
    double zSpeed;

    public AlignToAprilTag(PoseEstimatorSubsystem poseEstimatorSubsystem, DrivetrainSubsystem drivetrainSubsystem) {
        m_poseEstimaorSubsystem = poseEstimatorSubsystem;
        m_drivetrainSubsystem = drivetrainSubsystem;

        addRequirements(m_drivetrainSubsystem);
    }

    public void initialize() {
        rotationController.reset(m_poseEstimaorSubsystem.getAprilTagPoseToBot3d().getRotation().getY());
        rotationController.setGoal(0);
        rotationController.setTolerance(rotationTolerance);

        strafeController.reset(m_poseEstimaorSubsystem.getAprilTagPoseToBot3d().getX());
        strafeController.setGoal(0);
        strafeController.setTolerance(strafeTolerance);

        zController.reset(m_poseEstimaorSubsystem.getAprilTagPoseToBot3d().getZ());
        zController.setGoal(1.3);
        zController.setTolerance(zTolerance);
    }

    public void execute() {
        rotSpeed = 0;
        strafeSpeed = 0;
        zSpeed = 0;

        currAngle = m_poseEstimaorSubsystem.getAprilTagPoseToBot3d().getRotation().getY();
        currStrafeError = m_poseEstimaorSubsystem.getAprilTagPoseToBot3d().getX();
        currZError = m_poseEstimaorSubsystem.getAprilTagPoseToBot3d().getZ();
        
        if (LimelightHelpers.getTV("limelight")) {
            if (!rotationController.atGoal()) {
                rotSpeed = -rotationController.calculate(currAngle);
            }
            if (!strafeController.atGoal()) {
                strafeSpeed = -strafeController.calculate(currStrafeError);
            }
            if (!zController.atGoal()) {
                zSpeed = zController.calculate(currZError);
            }
        }

        m_drivetrainSubsystem.drive(new Translation2d(zSpeed, strafeSpeed), rotSpeed, true, false);
    }

    public void end(boolean interrupted) {
        m_drivetrainSubsystem.drive(new Translation2d(), 0, false, false);
    }

    public boolean isFinished() {
        return false;
    }
}