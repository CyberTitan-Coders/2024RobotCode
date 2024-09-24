package frc.robot.subsystems;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
// import edu.wpi.first.cscore.VideoSink;
// import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class CameraSubsystem extends SubsystemBase{

    UsbCamera camera0; 
    UsbCamera camera1; 
    //VideoSink server;
    NetworkTableEntry cameraSelection;


    public CameraSubsystem(){
        camera0 = CameraServer.startAutomaticCapture(0);
        camera1 = CameraServer.startAutomaticCapture(1);
       // server = CameraServer.getServer();

        cameraSelection = NetworkTableInstance.getDefault().getTable("").getEntry("CameraSelection");
    }


    public void selectCam(int cameraType){
        if(cameraType==0){
            cameraSelection.setString(camera0.getName());
             SmartDashboard.putString("camera", "Camera 0");
        }
        else if(cameraType == 1){
             cameraSelection.setString(camera1.getName());
             SmartDashboard.putString("camera", "Camera 1");
        }
        else 
        SmartDashboard.putString("camera", "error");
        
    }
   

}
