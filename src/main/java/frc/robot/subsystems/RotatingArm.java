package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
//import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
//import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;

import frc.robot.Constants;


public class RotatingArm extends SubsystemBase {

    private final CANSparkMax m_RightLift = new CANSparkMax(Constants.RoatingArmConstants.rightRotationMotorCanID,
            MotorType.kBrushless);
    private final CANSparkMax m_LeftLift = new CANSparkMax(Constants.RoatingArmConstants.leftRotationMotorCanID,
            MotorType.kBrushless);

    private final SparkMaxPIDController m_RightLiftPIDController = m_RightLift.getPIDController();
    private final RelativeEncoder m_RightLiftEncoder = m_RightLift.getEncoder();
    private final SparkMaxPIDController m_LeftLiftPIDController = m_LeftLift.getPIDController();
    private final RelativeEncoder m_LeftLiftEncoder = m_LeftLift.getEncoder();

    public RotatingArm() {
        m_RightLift.restoreFactoryDefaults();
        m_LeftLift.restoreFactoryDefaults();

        m_RightLift.setIdleMode(CANSparkMax.IdleMode.kBrake);
        m_LeftLift.setIdleMode(CANSparkMax.IdleMode.kBrake);

        m_RightLiftPIDController.setFeedbackDevice(m_RightLiftEncoder);
        m_LeftLiftPIDController.setFeedbackDevice(m_LeftLiftEncoder);

        m_RightLiftPIDController.setP(1);
        m_RightLiftPIDController.setI(0);
        m_RightLiftPIDController.setD(0);
        m_RightLiftPIDController.setFF(0);
        m_RightLiftPIDController.setOutputRange(0,Math.PI);

        m_LeftLiftPIDController.setP(1);
        m_LeftLiftPIDController.setI(0);
        m_LeftLiftPIDController.setD(0);
        m_LeftLiftPIDController.setFF(0);
        m_LeftLiftPIDController.setOutputRange(0,Math.PI);  

        //m_LeftLift.follow(m_RightLift, true); Possible solution to simply have the left mirror the right and cut down calculations

        m_RightLift.burnFlash();
        m_LeftLift.burnFlash();

    }

    public void periodic() {
     
    }

    public void stopMotors(){
        m_LeftLift.stopMotor();
        m_RightLift.stopMotor();
    }

    public CommandBase rotateCommand(double speed) {

        return this.startEnd(() ->  rotate(speed), () ->  stopMotors());

    }

    public void rotate(double speed){
        m_LeftLift.set(speed);
        m_RightLift.set(speed);
    }

    public CommandBase positionOneCommand(){
        return this.runOnce(this::positionOne);
    }

    public void positionOne() {
        m_RightLiftPIDController.setReference(Math.PI / 2, ControlType.kPosition);
        m_LeftLiftPIDController.setReference(Math.PI / 2, ControlType.kPosition);
    }

}
