// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team7520.robot.subsystems.Intake;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController.AccelStrategy;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team7520.robot.Constants;
import frc.team7520.robot.Constants.IntakeConstants.turn;

public class IntakePivot extends SubsystemBase {

  public CANSparkMax Turn = new CANSparkMax(turn.CAN_ID, MotorType.kBrushless);
  private RelativeEncoder TurnEncoder;
  private final SparkPIDController TurnPID;

  public Rotation2d DesiredPosition = Rotation2d.fromDegrees(turn.Shoot);

  private final static IntakePivot INSTANCE = new IntakePivot();

  @SuppressWarnings("WeakerAccess")
    public static IntakePivot getInstance() {
        return INSTANCE;
    }

  /** Creates a new ExampleSubsystem. */
  public IntakePivot() {
    this.TurnEncoder = Turn.getEncoder();
    TurnEncoder.setPosition(0);
    TurnEncoder.setPositionConversionFactor(turn.ConversionFactor);
    this.TurnPID = Turn.getPIDController();

    TurnPID.setP(Constants.IntakeConstants.turn.kP);
    TurnPID.setI(Constants.IntakeConstants.turn.kI);
    TurnPID.setD(Constants.IntakeConstants.turn.kD);

    TurnPID.setOutputRange(Constants.IntakeConstants.turn.OutputMin, Constants.IntakeConstants.turn.OutputMax);

    TurnPID.setSmartMotionMaxVelocity(Constants.IntakeConstants.turn.SmartMaxVel, Constants.IntakeConstants.turn.SlotID);
    TurnPID.setSmartMotionMinOutputVelocity(Constants.IntakeConstants.turn.SmartMinVel, Constants.IntakeConstants.turn.SlotID);
    TurnPID.setSmartMotionMaxAccel(Constants.IntakeConstants.turn.SmartAccel, Constants.IntakeConstants.turn.SlotID);
    TurnPID.setSmartMotionAllowedClosedLoopError(Constants.IntakeConstants.turn.SmartErr, Constants.IntakeConstants.turn.SlotID);

    Turn.setIdleMode(CANSparkMax.IdleMode.kBrake);
  }

  public void setRotation(Rotation2d position){
    DesiredPosition = position;
    }

  public Command Shoot() {
    return runOnce(
        () -> {
          setRotation(Rotation2d.fromDegrees(turn.Shoot));
        }).andThen(MoveIntake());
  }

  public Command Amp() {
    return runOnce(
        () -> {
          setRotation(Rotation2d.fromDegrees(turn.Amp));
        }).andThen(MoveIntake());
  }
  
  public Command Intake() {
    return runOnce(
        () -> {
          setRotation(Rotation2d.fromDegrees(turn.Floor));
        }).andThen(MoveIntake());
  }

  public Command Manual(DoubleSupplier turn) {
    return runOnce(
        () -> {
          double turnVal = turn.getAsDouble();

          if(Math.abs(turnVal) > 0.05) {
            DesiredPosition = DesiredPosition.plus(Rotation2d.fromDegrees(1 * turnVal));
          }
        }).andThen(MoveIntake());
  }

  public Command MoveIntake() {
    return runOnce(
        () -> {
          if(DesiredPosition.getDegrees() > Rotation2d.fromDegrees(turn.Floor).getDegrees()) {
            DesiredPosition = Rotation2d.fromDegrees(turn.Floor);
          }

          if(DesiredPosition.getDegrees() < Rotation2d.fromDegrees(turn.Shoot).getDegrees()) {
            DesiredPosition = Rotation2d.fromDegrees(turn.Shoot);
          }

          TurnPID.setReference(DesiredPosition.getRotations(), ControlType.kSmartMotion);
        }
    );
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("TurnEncoder", TurnEncoder.getPosition());
    SmartDashboard.putNumber("DesiredDeg", DesiredPosition.getDegrees());
    SmartDashboard.putNumber("DesiredRot", DesiredPosition.getRotations());
  }
}