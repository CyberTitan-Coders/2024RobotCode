package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class ShooterIntakeCommands {
    public static ShooterSubsystem m_shooter = ShooterSubsystem.getInstance();
    public static IntakeSubsystem m_intake = IntakeSubsystem.getInstance();

    private ShooterIntakeCommands(){
        throw new UnsupportedOperationException("This is a utility class");
    }

    // change for our specific robot .__. 
    public static Command shoot() {
        return Commands.runOnce(() -> {
            m_shooter.shootOut(1.0);
        })
                .andThen(Commands.waitSeconds(0.35))
                .andThen(Commands.runOnce(() -> {
                    m_intake.setIntake(-1.0);
                }).andThen(Commands.waitSeconds(0.6)
                        .andThen(Commands.runOnce(() -> {
                            m_intake.stopIntake();
                            m_shooter.stopShoot();
                        }))));
    }
}