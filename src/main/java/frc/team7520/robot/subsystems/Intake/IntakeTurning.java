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

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team7520.robot.Constants;

public class IntakeTurning extends SubsystemBase {

  public CANSparkMax turn = new CANSparkMax(Constants.IntakeConstants.turn.CAN_ID, MotorType.kBrushless);
  private RelativeEncoder turnEncoder;
  private final SparkPIDController turnPID;

  public double currentPosition = Position.SHOOT;

    class Position {
        public static double FLOOR = 51.78583526611328; //not exact
        public static double SHOOT = 0.238095209002495; //not exact
    }

  private final static IntakeTurning INSTANCE = new IntakeTurning();

  @SuppressWarnings("WeakerAccess")
    public static IntakeTurning getInstance() {
        return INSTANCE;
    }

  /** Creates a new ExampleSubsystem. */
  public IntakeTurning() {
    this.turnEncoder = turn.getEncoder();
    this.turnPID = turn.getPIDController();

    turnPID.setP(0.15);
    turnPID.setI(0);
    turnPID.setD(0);
    turnPID.setFF(0.1);
    turnPID.setOutputRange(-1, 1);

    turn.setIdleMode(CANSparkMax.IdleMode.kBrake);
  }

  public void setPosition(double position){
    currentPosition = position;
    }

  public Command Shoot() {
    return runOnce(
        () -> {
          setPosition(Position.SHOOT);
        }).andThen(MoveIntake());
  }
  
  public Command Intake() {
    return runOnce(
        () -> {
          setPosition(Position.FLOOR);
        }).andThen(MoveIntake());
  }

  public Command Manual(DoubleSupplier turn) {
    return runOnce(
        () -> {
          double turnVal = turn.getAsDouble();

          if(Math.abs(turnVal) > 0.05) {
            currentPosition = currentPosition + 1 * turnVal;
          }
        }).andThen(MoveIntake());
  }

  public Command MoveIntake() {
    return runOnce(
        () -> {
            turnPID.setReference(currentPosition, ControlType.kPosition);
        }
    );
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("TurnEncoder", turnEncoder.getPosition());
  }
}