// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team7520.robot.commands;

import frc.team7520.robot.Constants;
import frc.team7520.robot.subsystems.intake.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.BooleanSupplier;

/** An example command that uses an example subsystem. */
public class Intake extends Command {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final IntakeSubsystem intakeSubsystem;
    private final BooleanSupplier shootSup;
    private final BooleanSupplier shootPosSup;
    private final BooleanSupplier intakePosSup;
    private final BooleanSupplier ampPosSup;
    private final BooleanSupplier reverseSup;
    private final BooleanSupplier switchSup;

    public Constants.IntakeConstants.Position currPosition = Constants.IntakeConstants.Position.SHOOT;



    /**
     * Creates a new ExampleCommand.
     *
     * @param intakeSubsystem The subsystem used by this command.
     */
    public Intake(IntakeSubsystem intakeSubsystem, BooleanSupplier shootSup, BooleanSupplier shootPosSup,
                  BooleanSupplier intakePosSup, BooleanSupplier ampPosSup, BooleanSupplier reverseSup, BooleanSupplier switchSup) {
        this.intakeSubsystem = intakeSubsystem;
        this.shootSup = shootSup;
        this.shootPosSup = shootPosSup;
        this.intakePosSup = intakePosSup;
        this.ampPosSup = ampPosSup;
        this.reverseSup = reverseSup;
        this.switchSup = switchSup;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(intakeSubsystem);
    }

    public void handleWheels() {
        if (shootSup.getAsBoolean() && currPosition == Constants.IntakeConstants.Position.SHOOT) {
            intakeSubsystem.setSpeed(0.35, false);
            return;
        }
        if (shootSup.getAsBoolean() && currPosition == Constants.IntakeConstants.Position.AMP) {
            intakeSubsystem.setSpeed(0.525, false);
            return;
        }
        if (shootSup.getAsBoolean() && currPosition == Constants.IntakeConstants.Position.INTAKE) {
            if (switchSup.getAsBoolean()) {
                intakeSubsystem.setSpeed(-0.35, false);
            }
            else {
                intakeSubsystem.setSpeed(0, false);
            }
            return;
        }
        if(reverseSup.getAsBoolean()) {
            intakeSubsystem.setSpeed(-0.35);
            return;
        }
        intakeSubsystem.stop();
    }

    public void handlePosition() {
        if (shootPosSup.getAsBoolean()) {
            intakeSubsystem.setPosition(Constants.IntakeConstants.Position.SHOOT);
            currPosition = Constants.IntakeConstants.Position.SHOOT;
            return;
        }
        if (intakePosSup.getAsBoolean()) {
            intakeSubsystem.setPosition(Constants.IntakeConstants.Position.INTAKE);
            currPosition = Constants.IntakeConstants.Position.INTAKE;
            return;
        }
        if (ampPosSup.getAsBoolean()) {
            intakeSubsystem.setPosition(Constants.IntakeConstants.Position.AMP);
            currPosition = Constants.IntakeConstants.Position.AMP;
            return;
        }

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        handleWheels();
        handlePosition();

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
