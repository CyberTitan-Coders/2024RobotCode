package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.subsystems.IntakeSubsystem;

// run motor until note is detected --> immediately stop motors 
public class DetectNoteLow extends InstantCommand {
    private final IntakeSubsystem m_sensor;

    public DetectNoteLow(IntakeSubsystem sensor){
      m_sensor=sensor;
      addRequirements(m_sensor);
    }
    
    public void initialize(){
        m_sensor.detectStage1();
    }
    
}