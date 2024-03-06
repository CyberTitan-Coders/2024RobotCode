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

    public static CANSparkMax m_intake = new CANSparkMax(operatorStuff.kIntake_ID, MotorType.kBrushless);

    public IntakeSubsystem(){
        m_intake.setIdleMode(IdleMode.kBrake);
    }

    
    public void setIntakeSpeed(double speed){
        m_intake.set(speed);
        SmartDashboard.putNumber("Intake speed", speed);
       }

       public void stopIntake(){
        m_intake.set(0);
        SmartDashboard.putNumber("Intake has been stopped", 0);
       }
}
