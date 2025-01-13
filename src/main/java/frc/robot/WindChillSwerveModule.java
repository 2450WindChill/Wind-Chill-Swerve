package frc.robot;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.libs.OnboardModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class WindChillSwerveModule {
  public int moduleNumber;
  private Rotation2d lastAngle;

  private SparkMax angleMotor;
  private SparkMax driveMotor;

  private SparkBaseConfig angleConfig;
  private SparkBaseConfig driveConfig;

  private RelativeEncoder driveEncoder;
  private RelativeEncoder integratedAngleEncoder;
  private CANcoder angleEncoder;

  private final SparkClosedLoopController driveController;
  private final SparkClosedLoopController angleController;

  public WindChillSwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
    this.moduleNumber = moduleNumber;

    /* Angle Encoder Config */
    angleEncoder = new CANcoder(moduleConstants.cancoderID, "canivore");

    /* Angle Motor Config */
    angleMotor = new SparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);
    integratedAngleEncoder = angleMotor.getEncoder();
    angleController = angleMotor.getClosedLoopController();
    configAngleMotor();

    /* Drive Motor Config */
    driveMotor = new SparkMax(moduleConstants.driveMotorID, MotorType.kBrushless);
    driveEncoder = driveMotor.getEncoder();
    driveController = driveMotor.getClosedLoopController();
    configDriveMotor();

    lastAngle = getState().angle;

  }

  public void setDesiredState(SwerveModuleState desiredState, boolean isSlowMode) {
    desiredState = OnboardModuleState.optimize(desiredState, getState().angle);

    setAngle(desiredState);
    setSpeed(desiredState, isSlowMode);
  }

  private void setSpeed(SwerveModuleState desiredState, boolean isSlowMode) {
    double percentOutput = desiredState.speedMetersPerSecond / Constants.maxSpeed;
    if (!isSlowMode) {
      driveMotor.set(percentOutput);
    } else {
      driveMotor.set(percentOutput * 0.15);
    }
  }

  private void setAngle(SwerveModuleState desiredState) {
    // Prevent rotating module if speed is less then 1%. Prevents jittering.
    Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.maxSpeed * 0.01))
        ? lastAngle
        : desiredState.angle;

    double desiredAngle = angle.getDegrees();

    // while (desiredAngle < 0) {
    //   desiredAngle += 360;
    // }

    // while (desiredAngle > 360) {
    //   desiredAngle -= 360;
    // }

    SmartDashboard.putNumber("Desired Angle " + moduleNumber, desiredAngle);

    angleController.setReference(desiredAngle, SparkMax.ControlType.kPosition);
    lastAngle = angle;    
  }

  public void resetToAbsolute() {
    double absolutePosition = getCanCoderInDegrees();
    integratedAngleEncoder.setPosition(absolutePosition);
  }

  // Using getCancoder instead
  private Rotation2d getAngle() {
    return Rotation2d.fromDegrees(integratedAngleEncoder.getPosition());
  }

  private Rotation2d getAutoAngle() {
    return Rotation2d.fromDegrees(integratedAngleEncoder.getPosition());
  }

  public double getCanCoderInDegrees() {
    return getRawCanCoder() * 360.0;
  }

  public double getRawCanCoder() {
    return angleEncoder.getAbsolutePosition().getValueAsDouble();
  }

  public double getDriveEncoder() {
    return driveEncoder.getPosition();
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(driveEncoder.getVelocity(), getAngle());
  }

  public SwerveModuleState getAutoState() {
    return new SwerveModuleState(driveEncoder.getVelocity(), getAutoAngle());
  }

  private void configAngleMotor() {

    angleConfig = new SparkMaxConfig();
    angleConfig 
      .smartCurrentLimit(Constants.angleContinuousCurrentLimit)
      .inverted(Constants.angleInvert)
      .voltageCompensation(Constants.voltageComp);
    angleConfig.encoder
      .positionConversionFactor(Constants.angleConversionFactor);
    angleConfig.closedLoop
      // Wrap angle motors to stay within 0-360 degrees
      .positionWrappingEnabled(true)
      .positionWrappingMinInput(0.0)
      .positionWrappingMaxInput(360.0)
      .pidf(Constants.angleKP, Constants.angleKI, Constants.angleKD, Constants.angleKFF);
    
      angleMotor.configure(angleConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  private void configDriveMotor() {

    driveConfig = new SparkMaxConfig();
    driveConfig 
      .smartCurrentLimit(Constants.driveContinuousCurrentLimit)
      .idleMode(Constants.driveIdleMode)
      .inverted(Constants.driveInvert)
      .voltageCompensation(Constants.voltageComp);
    driveConfig.encoder
      .positionConversionFactor(Constants.driveConversionPositionFactor)
      .velocityConversionFactor(Constants.driveConversionVelocityFactor);
    driveConfig.closedLoop

      .pidf(Constants.driveKP, Constants.driveKI, Constants.driveKD, Constants.driveKFF);
    
      driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

      driveEncoder.setPosition(0.0);
  }

  // public SwerveModulePosition getPosition() {
  //   return new SwerveModulePosition(
  //       -((getDriveEncoder() / Constants.rotationsPerOneFoot) * Constants.feetToMeters),
  //       Rotation2d.fromDegrees(getCanCoderInDegrees()));
  // }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        -((getDriveEncoder() / Constants.rotationsPerOneFoot) * Constants.feetToMeters) * 0.9,
        Rotation2d.fromDegrees(getCanCoderInDegrees()));
  }


  public void setPosition(double position) {
    integratedAngleEncoder.setPosition(position);
  }
}
 