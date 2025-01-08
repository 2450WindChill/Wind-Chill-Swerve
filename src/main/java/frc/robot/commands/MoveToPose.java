// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class MoveToPose extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DrivetrainSubsystem m_drivetrainSubsystem;
  private final PoseEstimatorSubsystem m_poseEstimate;

  public double currentX;
  public double currentY;
  public double currentRotation;

  public double xError;
  public double yError;
  public double rotationError;

  public Pose2d m_targetPose;

  public Translation2d speeds;

  // PIDController xPIDController = new PIDController(0.1, 0.1, 0);
  // PIDController yPIDController = new PIDController(0.1, 0.1, 0);
  // PIDController rotationPIDController = new PIDController(0.1, 0.1, 0.0036);

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public MoveToPose(DrivetrainSubsystem drivetrainSubsystem, PoseEstimatorSubsystem poseEstimate, Pose2d targetPose) {
    m_drivetrainSubsystem = drivetrainSubsystem;
    m_poseEstimate = poseEstimate;
    m_targetPose = targetPose;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drivetrainSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // xPIDController.reset();
    // yPIDController.reset();
    // rotationPIDController.reset();

    // xPIDController.setSetpoint(m_targetPose.getX());
    // xPIDController.setSetpoint(m_targetPose.getY());
    // rotationPIDController.setSetpoint(m_targetPose.getRotation().getDegrees());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // Finds translation and rotation to desired pose
    System.out.println("MOVE TO POSE");
    xError = m_targetPose.getX() - m_poseEstimate.getBotX();
    yError = m_targetPose.getY() - m_poseEstimate.getBotY();
    rotationError = m_targetPose.getRotation().getDegrees() - m_poseEstimate.getBotRotation();

    SmartDashboard.putNumber("X Error", xError);
    SmartDashboard.putNumber("Y Error", yError);
    SmartDashboard.putNumber("Rotation Error", rotationError);

    // Calls .drive() with speeds and rotations towards desired pose
    m_drivetrainSubsystem.drive(
      new Translation2d(xError, yError).times(Constants.maxSpeed),
      rotationError * Constants.maxAngularVelocity,
      false,
      false
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrainSubsystem.drive(
      new Translation2d(0, 0),
      0,
      false,
      false
    );
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if ((Math.abs(rotationError) <= 5) && (Math.abs(xError) <= 0.05) && (Math.abs(yError) <= 0.05)) {
      System.out.println("MOVE TO POSE IS FINISHED");
      return true;
    } else {
      System.err.println("move to pose NOT finished");
      return false;
    }
  }
}
