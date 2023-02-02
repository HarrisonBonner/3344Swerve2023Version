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

public class Claw extends SubsystemBase{

    private final CANSparkMax m_clawMotor = new CANSparkMax(Constants.ClawConstants.clawMotorCanID,MotorType.kBrushless);

    public Claw(){
        m_clawMotor.setIdleMode(IdleMode.kBrake);
    }

    public CommandBase intake(){
        return this.startEnd(() -> m_clawMotor.set(-.5), () -> m_clawMotor.stopMotor());
    }

    public CommandBase outake(){
        return this.startEnd(() -> m_clawMotor.set(.5), () -> m_clawMotor.stopMotor());
    }
}
