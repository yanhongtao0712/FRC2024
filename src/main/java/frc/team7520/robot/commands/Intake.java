// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team7520.robot.commands;

import frc.team7520.robot.subsystems.Intake.IntakeSubsystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class Intake extends Command {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final IntakeSubsystem intakeSubsystem;

    private final boolean intake;
    private final boolean amp;

    /**
     * Creates a new ExampleCommand.
     *
     * @param intakeSubsystem The subsystem used by this command.
     */
    public Intake(IntakeSubsystem intakeSubsystem, boolean intake, boolean amp) {
        this.intakeSubsystem = intakeSubsystem;
        this.intake = intake;
        this.amp = amp;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(intakeSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (intake) {
            intakeSubsystem.setSpeed(0.35, false);
            return;
        }
        if (amp) {
            intakeSubsystem.setSpeed(-0.525, false);
            return;
        }
        intakeSubsystem.setSpeed(0);
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
