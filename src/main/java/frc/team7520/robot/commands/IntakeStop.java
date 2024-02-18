// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team7520.robot.commands;

import frc.team7520.robot.subsystems.Intake.IntakeRollers;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class IntakeStop extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final IntakeRollers IntakeRollersSubsystem;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public IntakeStop(IntakeRollers IntakeRollersSubsystem) {
    this.IntakeRollersSubsystem = IntakeRollersSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(IntakeRollersSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    IntakeRollersSubsystem.rollers.set(0);
  }
    

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}