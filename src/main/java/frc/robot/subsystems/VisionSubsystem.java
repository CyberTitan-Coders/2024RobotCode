package frc.robot.subsystems;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSink;


public class VisionSubsystem {

    UsbCamera camera0; 
    UsbCamera camera1; 
    VideoSink server;
    boolean camButtonEnabled = true; 
    int activeCamera = 0;

    public VisionSubsystem(){
        camera0 = CameraServer.startAutomaticCapture(0);
        camera1 = CameraServer.startAutomaticCapture(1);
        server = CameraServer.getServer();
        activeCamera = 0;
        server.setSource(camera0);
    }


    // Jeanelle fix this 
    public void buttonControl(){
        camButtonEnabled = false; 
        if(activeCamera == 0){
            activeCamera = 1;
            server.setSource(camera1);
        }
        else{
            activeCamera = 0; 
            server.setSource(camera0);
        }
        camButtonEnabled = true; 
    }


    
}
