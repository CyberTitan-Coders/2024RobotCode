package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ArmSubsystem;

public class StopArmMove extends InstantCommand{
    private final ArmSubsystem m_arm;  
    
    public StopArmMove(ArmSubsystem arm){
      m_arm = arm;
      addRequirements(m_arm);

    }

    public void initialize(){
        m_arm.stopArm();
    }
}