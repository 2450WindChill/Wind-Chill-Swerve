// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LauncherSubsystem extends SubsystemBase {
  public final CANSparkFlex bottomMotor = new CANSparkFlex(18, MotorType.kBrushless);
  public final CANSparkFlex topMotor = new CANSparkFlex(19, MotorType.kBrushless);

  /** Creates a new LauncherSubsystem. */
  public LauncherSubsystem() {
    Preferences.initDouble("LauncherSpeed", 0.2);
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }
  
  public void speakerTurnOnLauncher() {
    System.out.println("turn on");
    topMotor.set(0.7);
    bottomMotor.set(-0.7);
  }

  public void ampTurnOnLauncher() {
    System.out.println("turn on");
    topMotor.set(0.3);
    bottomMotor.set(-0.3);
  }

  public void turnOffLauncher() {
    System.out.println("turn off");
    topMotor.set(0);
    bottomMotor.set(0);
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a
   * digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}