package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
//import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
//import com.revrobotics.SparkMaxPIDController;
//import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
//import com.revrobotics.AbsoluteEncoder;
//import com.revrobotics.RelativeEncoder;

import frc.robot.Constants;

public class Intake extends SubsystemBase{

    private final CANSparkMax m_Intake = new CANSparkMax(Constants.IntakeConstants.intakeMotorCanID,MotorType.kBrushless);

    public Intake(){
        m_Intake.setIdleMode(IdleMode.kBrake);
    }

    public CommandBase intake(){
        return this.startEnd(() -> m_Intake.set(-.5), () -> m_Intake.stopMotor());
    }

    public CommandBase outake(){
        return this.startEnd(() -> m_Intake.set(.5), () -> m_Intake.stopMotor());
    }
}
