package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;

public class AlignToAprilTagSequential extends SequentialCommandGroup {

    PoseEstimatorSubsystem m_poseEstimaorSubsystem;
    DrivetrainSubsystem m_drivetrainSubsystem;

    public AlignToAprilTagSequential(PoseEstimatorSubsystem poseEstimatorSubsystem, DrivetrainSubsystem drivetrainSubsystem) {
        m_poseEstimaorSubsystem = poseEstimatorSubsystem;
        m_drivetrainSubsystem = drivetrainSubsystem;

        addCommands(
         new SquareToAprilTag(poseEstimatorSubsystem, drivetrainSubsystem),
         new StrafeToAprilTag(poseEstimatorSubsystem, drivetrainSubsystem),
         new ApproachAprilTag(poseEstimatorSubsystem, drivetrainSubsystem)
        );
    }
}