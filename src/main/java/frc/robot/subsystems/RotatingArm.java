package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.AbsoluteEncoder;

import frc.robot.Constants;

public class RotatingArm extends SubsystemBase {

    private final CANSparkMax m_Lift = new CANSparkMax(Constants.RotatingArmConstants.rotationMotorCanID,
            MotorType.kBrushless);
    private final CANSparkMax m_Wrist = new CANSparkMax(Constants.RotatingArmConstants.wristMotorCanId,
            MotorType.kBrushless);
    private AbsoluteEncoder m_LiftEncoder;
    private AbsoluteEncoder m_WristEncoder;

    private SparkMaxPIDController m_LiftPID;
    private SparkMaxPIDController m_WristPID;

    public RotatingArm() {

        m_Lift.setIdleMode(CANSparkMax.IdleMode.kBrake);
        m_LiftEncoder = m_Lift.getAbsoluteEncoder(Type.kDutyCycle);
        m_LiftPID = m_Lift.getPIDController();
        m_LiftPID.setFeedbackDevice(m_LiftEncoder);
        m_LiftPID.setP(Constants.RotatingArmConstants.liftPID.Pval);
        m_LiftPID.setI(Constants.RotatingArmConstants.liftPID.Ival);
        m_LiftPID.setD(Constants.RotatingArmConstants.liftPID.Dval);
        m_Lift.burnFlash();

        m_Wrist.setIdleMode(CANSparkMax.IdleMode.kBrake);
        m_WristEncoder = m_Wrist.getAbsoluteEncoder(Type.kDutyCycle);
        m_WristPID = m_Wrist.getPIDController();
        m_WristPID.setFeedbackDevice(m_WristEncoder);
        m_WristPID.setP(Constants.RotatingArmConstants.wristPID.Pval);
        m_WristPID.setI(Constants.RotatingArmConstants.wristPID.Ival);
        m_WristPID.setD(Constants.RotatingArmConstants.wristPID.Dval);
        m_Wrist.burnFlash();

    }

    public void periodic() {
    }



    public CommandBase rotateLiftCommand(double speed) {

        //return this.startEnd(() -> m_Lift.set(speed), () -> m_LiftPID.setReference(m_LiftEncoder.getPosition(), ControlType.kPosition));
        return this.startEnd(() ->  m_Lift.set(speed), () -> m_Lift.set(Constants.RotatingArmConstants.liftHoldSpeed));

    }

    public CommandBase rotateWristCommand(double speed){
        return this.startEnd(() -> m_Wrist.set(speed), () -> m_Wrist.set(Constants.RotatingArmConstants.wristHoldSpeed));
    }

    public void setArmPosition(double armAngle, double wristAngle){
        m_LiftPID.setReference(armAngle, CANSparkMax.ControlType.kPosition);
        m_WristPID.setReference(wristAngle, CANSparkMax.ControlType.kPosition);
    }

    public CommandBase armTopGoal() {
        return this.run( () -> setArmPosition(90, 0));
    }
    public CommandBase armMidGoal() {
        return this.run( () -> setArmPosition(45, 20));
    }
    public CommandBase armLowGoal() {
        return this.run( () -> setArmPosition(20, 40));
    }
    public CommandBase armFloor() {
        return this.run( () -> setArmPosition(10, 90));
    }



}
