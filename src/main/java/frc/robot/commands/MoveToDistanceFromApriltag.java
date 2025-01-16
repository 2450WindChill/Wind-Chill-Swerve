package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;

public class MoveToDistanceFromApriltag {

    PoseEstimatorSubsystem m_poseEstimatorSubsystem;
    DrivetrainSubsystem m_drivetrainSubsystem;

    // Need xy constraints
    ProfiledPIDController xController = new ProfiledPIDController(Constants.AutoConstants.X_kP, Constants.AutoConstants.X_kI, Constants.AutoConstants.X_kD, new Constraints(0, 0));
    ProfiledPIDController yController = new ProfiledPIDController(Constants.AutoConstants.Y_kP, Constants.AutoConstants.Y_kI, Constants.AutoConstants.Y_kD, new Constraints(0, 0));
    ProfiledPIDController thetaController = new ProfiledPIDController(Constants.AutoConstants.THETA_kP, Constants.AutoConstants.THETA_kI, Constants.AutoConstants.THETA_kD, Constants.AutoConstants.THETA_CONSTRAINTS);

    double targetX;
    double targetY;
    double targetTheta;


    // Drives the roboto to a pose realtive to an arpil tag
    // https://github.com/STMARobotics/frc-7028-2023/blob/main/src/main/java/frc/robot/commands/autonomous/DriveToPoseCommand.java
    public MoveToDistanceFromApriltag(DrivetrainSubsystem drivetrainSubsystem, PoseEstimatorSubsystem poseEstimatorSubsystem, Pose2d targetDistanceFromApriltag) {
        poseEstimatorSubsystem = m_poseEstimatorSubsystem;
        drivetrainSubsystem = m_drivetrainSubsystem;

        // Getting targets
        targetX = targetDistanceFromApriltag.getX();
        targetY = targetDistanceFromApriltag.getY();
        targetTheta = targetDistanceFromApriltag.getRotation().getRadians();

        // Tolerances, probably shoud be constants
        xController.setTolerance(0.1);
        yController.setTolerance(0.1);
        thetaController.setTolerance(Math.toRadians(5));
    }

    public void initialize() {
        // Set goals of each PID controller
        xController.setGoal(targetX);
        yController.setGoal(targetY);
        thetaController.setGoal(targetTheta);
    }

    public void execute() {
        // Run .calculate to find next speed
        double xpos = m_poseEstimatorSubsystem.getBotPoseToAprilTag().getX();
        double xSpeed = xController.calculate(xpos);

        double ypos = m_poseEstimatorSubsystem.getBotPoseToAprilTag().getY();
        double ySpeed = xController.calculate(ypos);

        double theta = m_poseEstimatorSubsystem.getBotPoseToAprilTag().getRotation().getAngle();
        double thetaSpeed = xController.calculate(theta);

        // Drive with those speeds
        m_drivetrainSubsystem.drive(new Translation2d(xSpeed, ySpeed), thetaSpeed, true, false);
    }

    public boolean isFinished() {
        // Check if all pid controllers are at goal
        return false;
    }

    public void end() {
        // Stop
    }
}
