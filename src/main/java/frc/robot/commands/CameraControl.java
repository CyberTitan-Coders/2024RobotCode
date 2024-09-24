package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.subsystems.CameraSubsystem;

public class CameraControl extends InstantCommand {
    private final CameraSubsystem m_CameraSubsystem;
    private final int m_cameraType;

    public CameraControl(CameraSubsystem cameraSubsystem, int cameraType){
        m_CameraSubsystem = cameraSubsystem;
        m_cameraType = cameraType;
    }

    public void initialize(){ 
        m_CameraSubsystem.selectCam(m_cameraType);
    }

}
