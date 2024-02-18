// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team7520.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;
import swervelib.parser.PIDFConfig;
import swervelib.telemetry.SwerveDriveTelemetry;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
    public static final Matter CHASSIS = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
    public static final double LOOP_TIME = 0.13; //s, 20ms + 110ms sprk max velocity lag

    public static class Robot {
        public static final double k_width = 26; // Inches
        public static final double k_length = 28; // Inches
      }
    
    public static final class Auton {

        public static final PIDFConfig TranslationPID = new PIDFConfig(0.7, 0, 0);
        public static final PIDFConfig angleAutoPID = new PIDFConfig(0.4, 0, 0.01);

        public static final double MAX_ACCELERATION = 2;
    }

    public static final class Drivebase {

        // Hold time on motor brakes when disabled
        public static final double WHEEL_LOCK_TIME = 10; // seconds
    }

    public static class OperatorConstants {
        // Joystick Ports
        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final int OPERATOR_CONTROLLER_PORT = 1;

        // Joystick Deadband
        public static final double LEFT_X_DEADBAND = 0.01;
        public static final double LEFT_Y_DEADBAND = 0.01;
        public static final double RIGHT_X_DEADBAND = 0.01;
        public static final double TURN_CONSTANT = 0.75;
    }

    public static class Swerve {
        public static final double DRIVE_GEAR_RATIO = 6.75;
        public static final double ANGLE_GEAR_RATIO = 150/7d;
    }

    public static class Telemetry {

        // change at comp to low
        public static final SwerveDriveTelemetry.TelemetryVerbosity SWERVE_VERBOSITY = SwerveDriveTelemetry.TelemetryVerbosity.HIGH;
    }

    public static class IntakeConstants {

    public static class turn {
      public static final int CAN_ID = 23;

      public static final double kP = 0.00022;
      public static final double kI = 0;
      public static final double kD = 0;
      public static final double kFF = 0.000156;

      public static final double OutputMax = 1;
      public static final double OutputMin = -1;

      public static final double SmartMaxVel = 3500;
      public static final double SmartMinVel = 0;
      public static final double SmartAccel = 2750;
      public static final double SmartErr = 0.4;
      public static final int SlotID = 0;
    }

    public static class rollers {
      public static final int CAN_ID = 22;
    }
  }

  public static class ShooterConstants {
    public static final int kShooterLeftMotorId = 20;
    public static final int kShooterRightMotorId = 21;

    public static final double kShooterP = 0.00001;
    public static final double kShooterI = 0.0;
    public static final double kShooterD = 0.0;

    public static final double kShooterMinOutput = 0;
    public static final double kShooterMaxOutput = 1;
  }
}
