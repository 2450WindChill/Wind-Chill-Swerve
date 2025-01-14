// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.MoveToPoseV1;
import frc.robot.libs.LimelightHelpers;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {
  // Subsystems
  public final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
  private final PoseEstimatorSubsystem m_poseEstimator = new PoseEstimatorSubsystem(m_drivetrainSubsystem);
  private final LimelightSubsystem m_limelightSubsystem = new LimelightSubsystem();

  // Controllers
  private final CommandXboxController m_driverController = new CommandXboxController(ControllerConstants.kDriverControllerPort);

  public SendableChooser<Command> m_chooser;

  public RobotContainer() {

    m_drivetrainSubsystem.setDefaultCommand(
        new DefaultDriveCommand(
            m_drivetrainSubsystem,
            () -> m_driverController.getLeftY(),
            () -> m_driverController.getLeftX(),
            () -> m_driverController.getRightX(),
            () -> Constants.isRobotCentric,
            m_driverController.leftTrigger()
          )
        );

    // Configures
    configureNamedCommands();
    configureBindings();
    configureLimelight();
    configureAutoChooser();
  }

  /*
   * Driver
   * x = zero gyro
   */

  private void configureBindings() {
    // Driver
    m_driverController.x().onTrue(Commands.runOnce(() -> m_drivetrainSubsystem.zeroGyro(), m_drivetrainSubsystem));
    m_driverController.y().onTrue(new MoveToPoseV1(m_drivetrainSubsystem, m_limelightSubsystem));
  }

  /*
   * Configures limelight to:
   * -Pipeline 0
   * -LEDs Off
   * -Proccesor Mode
   * -Pose relative to robot center (Meters and Degrees)
   */
  private void configureLimelight() {
    LimelightHelpers.setPipelineIndex("limelight", 0);
    LimelightHelpers.setLEDMode_ForceOff("limelight");
    // TODO: Find updated version
    //LimelightHelpers.setCameraMode_Processor("limelight");
    LimelightHelpers.setCameraPose_RobotSpace("limelight", 0, 0, 0, 0, 0, 0);
  }

  private void configureNamedCommands() {

  }

  // Creating different options for auto
  private void configureAutoChooser() {
    m_chooser = new SendableChooser<>();
    SmartDashboard.putData("Auto Mode", m_chooser);
  }

  // Auto command
  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }
}