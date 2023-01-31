package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;

import frc.robot.Constants;

//TODO set up pid loops for arm positions

public class RotatingArm extends SubsystemBase {

    private final CANSparkMax m_RightLift = new CANSparkMax(Constants.RoatingArmConstants.rightRotationMotor, MotorType.kBrushless);
    private final CANSparkMax m_LeftLift = new CANSparkMax(Constants.RoatingArmConstants.leftRotationMotor, MotorType.kBrushless);
    
    private final SparkMaxPIDController m_RightLiftPIDController = m_RightLift.getPIDController();

    private final RelativeEncoder m_RightLiftEncoder = m_RightLift.getEncoder();





    public RotatingArm(){
        m_RightLift.restoreFactoryDefaults();
        m_LeftLift.restoreFactoryDefaults();
        m_LeftLift.setInverted(true);

        m_RightLift.setIdleMode(CANSparkMax.IdleMode.kBrake);
        m_LeftLift.setIdleMode(CANSparkMax.IdleMode.kBrake);

        m_RightLiftPIDController.setFeedbackDevice(m_RightLiftEncoder);

    }

    public void periodic() {
    }

    public void rotate(boolean up){

        m_drivingPIDController.setReference(optimizedDesiredState.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity);

        if(up){
            m_LeftLift.set(0.50);
            m_RightLift.set(0.50);
        }
        else{
            m_LeftLift.set(-0.50);
            m_RightLift.set(-0.50);
        }
    }



}
