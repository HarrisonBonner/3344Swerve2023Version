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
    private final CANSparkMax m_Exstension = new CANSparkMax(Constants.RoatingArmConstants.extensionMotorCanId,
            MotorType.kBrushless);

    private final SparkMaxPIDController m_RightLiftPIDController = m_RightLift.getPIDController();
    private final RelativeEncoder m_RightLiftEncoder = m_RightLift.getEncoder();
    private final SparkMaxPIDController m_LeftLiftPIDController = m_LeftLift.getPIDController();
    private final RelativeEncoder m_LeftLiftEncoder = m_LeftLift.getEncoder();
    private final SparkMaxPIDController m_ExstensionPIDController = m_Exstension.getPIDController();
    private final RelativeEncoder m_ExstensionEncoder = m_Exstension.getEncoder();
    public RotatingArm() {
        m_RightLift.restoreFactoryDefaults();
        m_LeftLift.restoreFactoryDefaults();
        m_Exstension.restoreFactoryDefaults();

        m_RightLift.setIdleMode(CANSparkMax.IdleMode.kBrake);
        m_LeftLift.setIdleMode(CANSparkMax.IdleMode.kBrake);
        m_Exstension.setIdleMode(CANSparkMax.IdleMode.kBrake);

        m_RightLiftPIDController.setFeedbackDevice(m_RightLiftEncoder);
        m_LeftLiftPIDController.setFeedbackDevice(m_LeftLiftEncoder);
        m_ExstensionPIDController.setFeedbackDevice(m_ExstensionEncoder);

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

        m_ExstensionPIDController.setP(1);
        m_ExstensionPIDController.setI(0);
        m_ExstensionPIDController.setD(0);
        m_ExstensionPIDController.setFF(0);
        m_ExstensionPIDController.setOutputRange(0, 4 * Math.PI);  
        //m_LeftLift.follow(m_RightLift, true); Possible solution to simply have the left mirror the right and cut down calculations

        m_RightLift.burnFlash();
        m_LeftLift.burnFlash();
        m_Exstension.burnFlash();

    }

    public void periodic() {
     
    }

    public void stopRotationMotors(){
        m_LeftLift.stopMotor();
        m_RightLift.stopMotor();
    }

    public CommandBase rotateCommand(double speed) {

        return this.startEnd(() ->  rotate(speed), () ->  stopRotationMotors());

    }

    public void rotate(double speed){
        m_LeftLift.set(speed);
        m_RightLift.set(speed);
    }

    public CommandBase rotationPositionOneCommand(){
        return this.runOnce(this::rotationPositionOne);
    }

    public void rotationPositionOne() {
        m_RightLiftPIDController.setReference(Math.PI / 2, ControlType.kPosition);
        m_LeftLiftPIDController.setReference(Math.PI / 2, ControlType.kPosition);
    }

    public void slideArm(double speed){
        m_Exstension.set(speed);
    }

    public CommandBase extendArmCommand(){
        return this.startEnd(() -> slideArm(Constants.RoatingArmConstants.extensionMaxSpeed), () -> m_Exstension.stopMotor());
    }

    public CommandBase retractArmCommand(){
        return this.startEnd(() -> slideArm(Constants.RoatingArmConstants.extensionMaxSpeed * -1), () -> m_Exstension.stopMotor());
    }
}
