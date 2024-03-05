package frc.robot.subsystems;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.operatorStuff;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
//import edu.wpi.first.wpilibj.Ultrasonic;
// import com.revrobotics.SparkAbsoluteEncoder.Type;
// import com.revrobotics.AbsoluteEncoder;
// import com.revrobotics.SparkRelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;


public class IntakeSubsystem {

    public static CANSparkMax m_intake; 
    private static IntakeSubsystem m_Instance = null;

    public IntakeSubsystem(int intakeID){
        m_intake = new CANSparkMax(intakeID, MotorType.kBrushless);
        m_intake.setIdleMode(IdleMode.kBrake);
    }


    

    public void setIntake(double speed){
        m_intake.set(speed);
        SmartDashboard.putNumber("Intake speed", speed);
       }

    public void stopIntake(){
        m_intake.set(0);
        SmartDashboard.putNumber("Stop intake", 0);
    }

    public void noteIn(){
        // jeanelle please write this 
        // run intake motor until the note is detected and stop motors 
        // probably use ultrasonic sensor -- need to figure out where to put it on robot 
    }

    public static IntakeSubsystem getInstance(){
        if(m_Instance == null){
            m_Instance = new IntakeSubsystem(operatorStuff.kIntake_ID);
        }
        return m_Instance; 
    }
    
}
