package frc.robot.subsystems;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSink;


public class CameraSubsystem {

    UsbCamera camera0; 
   // UsbCamera camera1; 
    VideoSink server;
    // boolean camButtonEnabled = true; 
    int activeCamera;

    public CameraSubsystem(){
        camera0 = CameraServer.startAutomaticCapture(0);
        //camera1 = CameraServer.startAutomaticCapture(1);
        server = CameraServer.getServer();
        activeCamera = 0;
        server.setSource(camera0);
    }


    // public void cameraControl(){
    //     // activeCamera++;
        
    //     // if(activeCamera%2 == 0){
    //         server.setSource(camera0);
    //     // }
    //     // else 
    //     //     server.setSource(camera1);
    // }


    public void periodic(){
        server.setSource(camera0);
    }

    // public void buttonControl(){
    //     camButtonEnabled = false; 
    //     if(activeCamera == 0){
    //         activeCamera = 1;
    //         server.setSource(camera1);
    //     }
    //     else{
    //         activeCamera = 0; 
    //         server.setSource(camera0);
    //     }
    //     camButtonEnabled = true; 
    // }


    
}
