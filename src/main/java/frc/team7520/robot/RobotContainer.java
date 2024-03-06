// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team7520.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.team7520.robot.Constants.OperatorConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.team7520.robot.commands.AbsoluteDrive;
import frc.team7520.robot.commands.Intake;
import frc.team7520.robot.commands.Shooter;
import frc.team7520.robot.commands.TeleopDrive;
import frc.team7520.robot.subsystems.Intake.IntakeSubsystem;
import frc.team7520.robot.subsystems.LED;
import frc.team7520.robot.subsystems.shooter.ShooterSubsystem;
import frc.team7520.robot.subsystems.swerve.SwerveSubsystem;
import frc.team7520.robot.util.PathPlannerHelper;
import frc.team7520.robot.util.RoutePlanner;

import java.io.File;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{
    // Subsystems
    private final SwerveSubsystem drivebase;
    private final ShooterSubsystem shooterSubsystem = ShooterSubsystem.getInstance();

    private final IntakeSubsystem intakeSubsystem = IntakeSubsystem.getInstance();

    // Replace with CommandPS4Controller or CommandJoystick if needed
    private final XboxController driverController =
            new XboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);

    private final XboxController operatorController =
            new XboxController(OperatorConstants.OPERATOR_CONTROLLER_PORT);

    // Create path chooser for testing purpose, verify if the swerve could move as expected
    SendableChooser<Command> pathChooser = new SendableChooser<>();
    // Create autoChooser for automomous
    SendableChooser<Command> autoChooser = new SendableChooser<>();
    public RoutePlanner myRoute = new RoutePlanner(pathChooser, autoChooser);

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer()
    {
        LED.getInstance().test();

        CameraServer.startAutomaticCapture();

        // Configure the trigger bindings
        // Load different setting for Swerve 2 or 3
        if(Constants.Drivebase.SWERVE_BASE_NUMBER==3)
        {
                drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                "swerve/neo"));
                
        }
        else
        {
                drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                "swerve2/neo"));       
        }
        SmartDashboard.putNumber("Swerve Base:", Constants.Drivebase.SWERVE_BASE_NUMBER);
                // Configure the trigger bindings




        // Left joystick is the angle of the robot
        AbsoluteDrive closedAbsoluteDrive = new AbsoluteDrive(drivebase,
                // Applies deadbands and inverts controls because joysticks
                // are back-right positive while robot
                // controls are front-left positive
                () -> -MathUtil.applyDeadband(driverController.getLeftY(),
                        OperatorConstants.LEFT_Y_DEADBAND),
                () -> -MathUtil.applyDeadband(driverController.getLeftX(),
                        OperatorConstants.LEFT_X_DEADBAND),
                () -> driverController.getRightX(),
                () -> driverController.getRightY(),
                driverController::getRightBumper,
                driverController::getLeftBumper
        );

        Shooter shooter = new Shooter(shooterSubsystem,
                operatorController::getLeftTriggerAxis,
                operatorController::getLeftBumper
        );

        Intake intake = new Intake(intakeSubsystem,
                operatorController::getRightBumper,
                operatorController::getYButton,
                operatorController::getAButton,
                operatorController::getBButton,
                operatorController::getXButton
        );

        // Old drive method
        // like in video games
        // Easier to learn, harder to control
        // Not tested not used
        TeleopDrive simClosedFieldRel = new TeleopDrive(drivebase,
                () -> MathUtil.applyDeadband(driverController.getLeftY(),
                        OperatorConstants.LEFT_Y_DEADBAND),
                () -> MathUtil.applyDeadband(driverController.getLeftX(),
                        OperatorConstants.LEFT_X_DEADBAND),
                () -> driverController.getRawAxis(2), () -> true);

        drivebase.setDefaultCommand(closedAbsoluteDrive);
        shooterSubsystem.setDefaultCommand(shooter);
        intakeSubsystem.setDefaultCommand(intake);

        // Display Chooser 
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Mode", autoChooser);
        SmartDashboard.putData("Test Route", pathChooser); 
        myRoute.ConfigureAutoPathProfile();
        myRoute.ConfigureManualPathProfile();
        
        configureBindings();
    }

    /**
     * Use this method to define named commands for use in {@link PathPlannerAuto}
     *
     */
    private void registerNamedCommands()
    {
        // Example
        NamedCommands.registerCommand("Shoot", new WaitCommand(1));
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
     * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings()
    {
        // Zero gyro
        new JoystickButton(driverController, XboxController.Button.kA.value)
                //.onTrue(new InstantCommand(drivebase::zeroGyro));
                .onTrue(new InstantCommand(drivebase::myReset));
        // X/Lock wheels
        /*
        new JoystickButton(driverController, XboxController.Button.kX.value)
                .whileTrue(new RepeatCommand(new InstantCommand(drivebase::lock)));
         */
        new JoystickButton(driverController, XboxController.Button.kX.value)
                .onTrue(
                        PathPlannerHelper.GoToCommand_AprilTag(
                                drivebase, 7
                                )
                        );
        // Run the command from path Chooser list
        new JoystickButton(driverController, XboxController.Button.kY.value)
                .onTrue(myRoute.getPathPlanerRoute());
    }


    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand()
    {
       // return drivebase.getPPAutoCommand("Demo1", true);
       return autoChooser.getSelected();
    }
}
