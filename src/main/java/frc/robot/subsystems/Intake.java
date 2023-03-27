package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax.IdleMode;


import frc.robot.Constants;

public class Intake extends SubsystemBase{

    private final CANSparkMax m_Intake = new CANSparkMax(Constants.IntakeConstants.intakeMotorCanID,MotorType.kBrushless);

    public Intake(){
        m_Intake.setIdleMode(IdleMode.kBrake);
        m_Intake.setSmartCurrentLimit(Constants.IntakeConstants.intakeSmartCurrentLimit);
        m_Intake.burnFlash();
    }

    public CommandBase intake(){
        return this.startEnd(() -> m_Intake.set(Constants.IntakeConstants.intakeMaxSpeed), () -> m_Intake.stopMotor());
    }

    public CommandBase outake(){
        return this.startEnd(() -> m_Intake.set(-1 * Constants.IntakeConstants.intakeMaxSpeed), () -> m_Intake.stopMotor());
    }

    public CommandBase outakeConeCommand(){
        return this.runOnce(() -> m_Intake.set(Constants.IntakeConstants.intakeMaxSpeed));
    }

    public CommandBase outakeCubeCommand(){
        return this.runOnce(() -> m_Intake.set(-1 * Constants.IntakeConstants.intakeMaxSpeed));
    } 

    public CommandBase stopIntake(){
        return this.runOnce(() -> m_Intake.stopMotor());
    }
}
