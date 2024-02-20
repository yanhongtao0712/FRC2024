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

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team7520.robot.Constants.IntakeConstants.pivotConstants;

public class IntakePivot extends SubsystemBase {

  public CANSparkMax pivot = new CANSparkMax(pivotConstants.CAN_ID, MotorType.kBrushless);
  private RelativeEncoder pivotEncoder;
  private final SparkPIDController pivotPID;

  public Rotation2d DesiredPosition = Rotation2d.fromDegrees(pivotConstants.Shoot);

  private final static IntakePivot INSTANCE = new IntakePivot();

  @SuppressWarnings("WeakerAccess")
    public static IntakePivot getInstance() {
        return INSTANCE;
    }

  /** Creates a new ExampleSubsystem. */
  public IntakePivot() {
    this.pivotEncoder = pivot.getEncoder();
    pivotEncoder.setPosition(0);
    pivotEncoder.setPositionConversionFactor(pivotConstants.ConversionFactor);
    this.pivotPID = pivot.getPIDController();

    pivotPID.setP(pivotConstants.kP);
    pivotPID.setI(pivotConstants.kI);
    pivotPID.setD(pivotConstants.kD);
    pivotPID.setFF(pivotConstants.kFF);
    pivotPID.setOutputRange(pivotConstants.OutputMin, pivotConstants.OutputMax);

    pivotPID.setSmartMotionMaxVelocity(pivotConstants.SmartMaxVel, pivotConstants.SlotID);
    pivotPID.setSmartMotionMinOutputVelocity(pivotConstants.SmartMinVel, pivotConstants.SlotID);
    pivotPID.setSmartMotionMaxAccel(pivotConstants.SmartAccel, pivotConstants.SlotID);
    pivotPID.setSmartMotionAllowedClosedLoopError(pivotConstants.SmartErr, pivotConstants.SlotID);

    pivot.setIdleMode(CANSparkMax.IdleMode.kBrake);
  }

  public void setRotation(Rotation2d position){
    DesiredPosition = position;
    }

  public Command Shoot() {
    return runOnce(
        () -> {
          setRotation(Rotation2d.fromDegrees(pivotConstants.Shoot));
        }).andThen(MoveIntake());
  }

  public Command Amp() {
    return runOnce(
        () -> {
          setRotation(Rotation2d.fromDegrees(pivotConstants.Amp));
        }).andThen(MoveIntake());
  }
  
  public Command Intake() {
    return runOnce(
        () -> {
          setRotation(Rotation2d.fromDegrees(pivotConstants.Floor));
        }).andThen(MoveIntake());
  }

  public Command Manual(DoubleSupplier pivot) {
    return runOnce(
        () -> {
          double pivotVal = pivot.getAsDouble();

          if(Math.abs(pivotVal) > 0.05) {
            DesiredPosition = DesiredPosition.plus(Rotation2d.fromDegrees(1 * pivotVal));
          }
        }).andThen(MoveIntake());
  }

  public Command MoveIntake() {
    return runOnce(
        () -> {
          if(DesiredPosition.getDegrees() > Rotation2d.fromDegrees(pivotConstants.Floor).getDegrees()) {
            DesiredPosition = Rotation2d.fromDegrees(pivotConstants.Floor);
          }

          if(DesiredPosition.getDegrees() < Rotation2d.fromDegrees(pivotConstants.Shoot).getDegrees()) {
            DesiredPosition = Rotation2d.fromDegrees(pivotConstants.Shoot);
          }

          pivotPID.setReference(DesiredPosition.getRotations(), ControlType.kSmartMotion);
        }
    );
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("pivotEncoder", pivotEncoder.getPosition());
    SmartDashboard.putNumber("DesiredDeg", DesiredPosition.getDegrees());
    SmartDashboard.putNumber("DesiredRot", DesiredPosition.getRotations());
  }
}