package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ArmSubsystem;

public class SetArmAngleDown extends InstantCommand{
    private final ArmSubsystem m_arm; 
    private final double m_targetAngle; 
    
    public SetArmAngleDown(ArmSubsystem arm, double targetAngle){
      m_arm = arm;
      m_targetAngle = targetAngle;
      addRequirements(m_arm);

    }

    public void initialize(){
        m_arm.setShooterAngleDown(m_targetAngle);
    }
}