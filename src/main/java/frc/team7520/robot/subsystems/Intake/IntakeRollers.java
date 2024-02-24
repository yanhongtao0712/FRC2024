// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team7520.robot.subsystems.Intake;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team7520.robot.Constants;

public class IntakeRollers extends SubsystemBase {

  public CANSparkMax rollers = new CANSparkMax(Constants.IntakeConstants.PivotConstants.CAN_ID, MotorType.kBrushless);

  private final static IntakeRollers INSTANCE = new IntakeRollers();

  @SuppressWarnings("WeakerAccess")
    public static IntakeRollers getInstance() {
        return INSTANCE;
    }

  /** Creates a new ExampleSubsystem. */
  public IntakeRollers() {
    rollers.setInverted(true);
  }

  public Command Intake() {
    return run(
      () -> {
        rollers.set(0.35);
      });
  }

  public Command Amp() {
    return run(
        () -> {
          rollers.set(-0.525);
        });
  }

  public Command Stop() {
    return run(
        () -> {
          rollers.set(0);
        });
  }

  public Command ControlledShooting(DoubleSupplier ShootVal) {
    return run(
        () -> {
          rollers.set(-0.9 * ShootVal.getAsDouble());
        });
  }
}
