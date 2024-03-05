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

public class ShooterSubsystem {

    private CANSparkMax m_topShooter; 
    private CANSparkMax m_bottomShooter; 

    private static ShooterSubsystem m_Instance = null;

    public ShooterSubsystem(int topShooterID, int botShooterID){
        m_topShooter = new CANSparkMax(topShooterID, MotorType.kBrushless);
        m_bottomShooter = new CANSparkMax(botShooterID, MotorType.kBrushless);

        m_topShooter.setIdleMode(IdleMode.kBrake);
        m_bottomShooter.setIdleMode(IdleMode.kBrake);
    }

    public void shootIn(double speed){
        m_topShooter.set(speed);
        m_bottomShooter.set(speed);

        SmartDashboard.putNumber("top shooter speed", speed);
        SmartDashboard.putNumber("bottom shooter speed", speed);
    }

       public void shootOut(double speed){
        m_topShooter.set(-speed);
        m_bottomShooter.set(-speed);

        SmartDashboard.putNumber("top shooter speed", -speed);
        SmartDashboard.putNumber("bottom shooter speed", -speed);
    }

    public void stopShoot(){
        m_topShooter.set(0);
        m_bottomShooter.set(0);
        SmartDashboard.putNumber("Motors stopped", 0);
    }

    public static ShooterSubsystem getInstance(){
        if(m_Instance == null){
            m_Instance = new ShooterSubsystem(operatorStuff.kTop_ID, operatorStuff.kBot_ID);
        }
        return m_Instance; 
    }
    
}
