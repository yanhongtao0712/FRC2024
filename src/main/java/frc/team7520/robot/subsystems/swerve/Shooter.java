package frc.team7520.robot.subsystems.swerve;

import com.revrobotics.*;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.jni.CANSparkMaxJNI;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team7520.robot.Constants;
import frc.team7520.robot.RobotContainer;

public class Shooter extends SubsystemBase {

  /*-------------------------------- Private instance variables ---------------------------------*/
  private static Shooter mInstance;
  private PeriodicIO mPeriodicIO;

  public static Shooter getInstance() {
    if (mInstance == null) {
      mInstance = new Shooter();
    }
    return mInstance;
  }

  private CANSparkMax mLeftShooterMotor;
  private CANSparkMax mRightShooterMotor;

  private SparkPIDController mLeftShooterPID;
  private SparkPIDController mRightShooterPID;

  private RelativeEncoder mLeftShooterEncoder;
  private RelativeEncoder mRightShooterEncoder;

  private SlewRateLimiter mSpeedLimiter = new SlewRateLimiter(1);

  private Shooter() {
    super("Shooter");

    mPeriodicIO = new PeriodicIO();

    mLeftShooterMotor = new CANSparkMax(Constants.kShooterLeftMotorId, MotorType.kBrushless);
    mRightShooterMotor = new CANSparkMax(Constants.kShooterRightMotorId, MotorType.kBrushless);
    mLeftShooterMotor.restoreFactoryDefaults();
    mRightShooterMotor.restoreFactoryDefaults();

    mLeftShooterPID = mLeftShooterMotor.getPIDController();
    mLeftShooterPID.setP(Constants.kShooterP);
    mLeftShooterPID.setI(Constants.kShooterI);
    mLeftShooterPID.setD(Constants.kShooterD);
    mLeftShooterPID.setOutputRange(Constants.kShooterMinOutput, Constants.kShooterMaxOutput);

    mRightShooterPID = mRightShooterMotor.getPIDController();
    mRightShooterPID.setP(Constants.kShooterP);
    mRightShooterPID.setI(Constants.kShooterI);
    mRightShooterPID.setD(Constants.kShooterD);
    mRightShooterPID.setOutputRange(Constants.kShooterMinOutput, Constants.kShooterMaxOutput);

    mLeftShooterEncoder = mLeftShooterMotor.getEncoder();
    mRightShooterEncoder = mRightShooterMotor.getEncoder();

    mLeftShooterMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
    mRightShooterMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);

    mLeftShooterMotor.setInverted(false);
    mRightShooterMotor.setInverted(true);

  }

  private static class PeriodicIO {
    double shooter_rpm = 0.0;
  }

  /*-------------------------------- Generic Subsystem Functions --------------------------------*/

  @Override
  public void periodic() {
    mLeftShooterPID.setReference(mPeriodicIO.shooter_rpm, ControlType.kVelocity);
    mRightShooterPID.setReference(mPeriodicIO.shooter_rpm, ControlType.kVelocity);
  }
  
  /*---------------------------------- Custom Public Functions ----------------------------------*/

  public void SetSpeed(double rpm) {
    mPeriodicIO.shooter_rpm = rpm;
  }

  public void stopShooter() {
    mPeriodicIO.shooter_rpm = 0.0;
  }

  /*---------------------------------- Custom Private Functions ---------------------------------*/
}
