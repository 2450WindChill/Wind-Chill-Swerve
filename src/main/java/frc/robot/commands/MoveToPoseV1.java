// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class MoveToPoseV1 extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DrivetrainSubsystem m_drivetrainSubsystem;
  private final LimelightSubsystem m_limelightSubsystem;

  // PIDController xPIDController = new PIDController(0.1, 0.1, 0);
  // PIDController yPIDController = new PIDController(0.1, 0.1, 0);
  // PIDController rotationPIDController = new PIDController(0.1, 0.1, 0.0036);

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public MoveToPoseV1(DrivetrainSubsystem drivetrainSubsystem, LimelightSubsystem limelightSubsystem) {
    m_drivetrainSubsystem = drivetrainSubsystem;
    m_limelightSubsystem = limelightSubsystem;

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

    final var rot_limelight = m_limelightSubsystem.limelight_aim_proportional();
    double rot = rot_limelight;

    final var forward_limelight = m_limelightSubsystem.limelight_range_proportional();
     double xSpeed = forward_limelight;


    // Calls .drive() with speeds and rotations towards desired pose
    m_drivetrainSubsystem.drive(
      new Translation2d(xSpeed, 0).times(Constants.maxSpeed),
    //   rotationError * Constants.maxAngularVelocity,
    rot,
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
    // TODO: make finish logic!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! pls
    return true;
  }
}
