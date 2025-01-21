package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.libs.LimelightHelpers;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;

public class SquareToAprilTag extends Command {

    ProfiledPIDController controller = new ProfiledPIDController(5, 0, 0, new Constraints(Constants.maxAngularVelocity, 4));

    PoseEstimatorSubsystem m_poseEstimaorSubsystem;
    DrivetrainSubsystem m_drivetrainSubsystem;

    double tolerance = Math.toRadians(3);

    double currAngle;

    double rotSpeed;

    public SquareToAprilTag(PoseEstimatorSubsystem poseEstimatorSubsystem, DrivetrainSubsystem drivetrainSubsystem) {
        m_poseEstimaorSubsystem = poseEstimatorSubsystem;
        m_drivetrainSubsystem = drivetrainSubsystem;

        addRequirements(m_drivetrainSubsystem);
    }

    public void initialize() {
        controller.reset(m_poseEstimaorSubsystem.getAprilTagPoseToBot3d().getRotation().getY());
        controller.setGoal(0);
        controller.setTolerance(tolerance);
    }

    public void execute() {
        // rotSpeed = 0;
        currAngle = m_poseEstimaorSubsystem.getAprilTagPoseToBot3d().getRotation().getY();
        
        // if (LimelightHelpers.getTV("limelight") && !controller.atGoal()) {
            rotSpeed = -controller.calculate(currAngle);
        // }

        m_drivetrainSubsystem.drive(new Translation2d(), rotSpeed, true, false);
        SmartDashboard.putNumber("rotSpeed", rotSpeed);
        SmartDashboard.putNumber("currAngle", currAngle);
    }

    public void end(boolean interrupted) {
        m_drivetrainSubsystem.drive(new Translation2d(), 0, false, false);
    }

    public boolean isFinished() {
        return (!LimelightHelpers.getTV("limelight") || controller.atGoal());
    }
}