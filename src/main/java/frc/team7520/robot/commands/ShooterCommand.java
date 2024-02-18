package frc.team7520.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.team7520.robot.subsystems.swerve.Shooter;

public class ShooterCommand extends Command{
    private final Shooter m_Shooter;
    public XboxController m_Controller;

    public ShooterCommand(Shooter shooter, XboxController controller){
        this.m_Controller = controller;
        this.m_Shooter = shooter;
        addRequirements(shooter);

    }
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }
    double speed = 0;
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if(m_Controller.getAButton() == true){
            speed = 0.5;
          } else if(m_Controller.getBButton() == true){
            speed = 1;
          } else if(m_Controller.getYButton() == true){
            speed = -0.2;
          } else{
            speed = 0;
          }
          speed = MathUtil.clamp(speed, -0.3, 1);
          m_Shooter.setSpeed(speed);
          // shooter1.set(operatorController.getLeftY());
          // shooter2.set(operatorController.getRightY());
      
          // SmartDashboard.putNumber("Shooter 1 velocity", shooter1.getEncoder().getVelocity());
          // SmartDashboard.putNumber("Shooter 2 velocity", shooter2.getEncoder().getVelocity());
          SmartDashboard.putNumber("controller", -(m_Controller.getLeftY()));
          SmartDashboard.putNumber("speed", speed);
          SmartDashboard.putBoolean("AButton", m_Controller.getAButton());
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
