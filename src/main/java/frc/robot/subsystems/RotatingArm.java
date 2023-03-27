package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.AbsoluteEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class RotatingArm extends SubsystemBase {

    private final CANSparkMax m_Lift = new CANSparkMax(Constants.RotatingArmConstants.rotationMotorCanID,
            MotorType.kBrushless);
    private final CANSparkMax m_Wrist = new CANSparkMax(Constants.RotatingArmConstants.wristMotorCanId,
            MotorType.kBrushless);
    private AbsoluteEncoder m_LiftEncoder;
    private AbsoluteEncoder m_WristEncoder;

    public static SparkMaxPIDController m_LiftPID;
    public static  SparkMaxPIDController m_WristPID;

    public RotatingArm() {
        //Config controllers, encoder, and pid
        //TODO wrist and arm softlimits
        m_Lift.restoreFactoryDefaults();
        m_Lift.setIdleMode(CANSparkMax.IdleMode.kBrake);
        //m_Lift.getAlternateEncoder(8192);
        m_LiftEncoder = m_Lift.getAbsoluteEncoder(Type.kDutyCycle);
        m_LiftEncoder.setInverted(Constants.RotatingArmConstants.liftEncoderInverted);
        m_LiftEncoder.setPositionConversionFactor(360);
        m_LiftPID = m_Lift.getPIDController();
        m_LiftPID.setFeedbackDevice(m_LiftEncoder);
        m_LiftPID.setP(Constants.RotatingArmConstants.liftPID.Pval);
        m_LiftPID.setI(Constants.RotatingArmConstants.liftPID.Ival);
        m_LiftPID.setD(Constants.RotatingArmConstants.liftPID.Dval);
        // m_Lift.enableSoftLimit(SoftLimitDirection.kForward, false);
        // m_Lift.enableSoftLimit(SoftLimitDirection.kReverse, false);
        // m_Lift.setSoftLimit(SoftLimitDirection.kForward, 120);
        // m_Lift.setSoftLimit(SoftLimitDirection.kReverse, 0);
        m_Lift.burnFlash();

        m_Wrist.restoreFactoryDefaults();
        m_Wrist.setIdleMode(CANSparkMax.IdleMode.kBrake);
        //m_Wrist.getAlternateEncoder(8192);
        m_WristEncoder = m_Wrist.getAbsoluteEncoder(Type.kDutyCycle);
        m_WristEncoder.setInverted(Constants.RotatingArmConstants.wristEncoderInverted);
        m_WristEncoder.setPositionConversionFactor(360);
        m_WristPID = m_Wrist.getPIDController();
        m_WristPID.setFeedbackDevice(m_WristEncoder);
        m_WristPID.setP(Constants.RotatingArmConstants.wristPID.Pval);
        m_WristPID.setI(Constants.RotatingArmConstants.wristPID.Ival);
        m_WristPID.setD(Constants.RotatingArmConstants.wristPID.Dval);
        // m_Wrist.enableSoftLimit(SoftLimitDirection.kForward, false);
        // m_Wrist.enableSoftLimit(SoftLimitDirection.kReverse, false);
        // m_Wrist.setSoftLimit(SoftLimitDirection.kForward, 120);
        // m_Wrist.setSoftLimit(SoftLimitDirection.kReverse, 0);
        m_Wrist.burnFlash();

    }
    //TODO check if updates values
    public void periodic() {
        SmartDashboard.putNumber("Wrist Angle", m_WristEncoder.getPosition());
        SmartDashboard.putNumber("Lift Angle",  m_LiftEncoder.getPosition());
    }



    public CommandBase rotateLiftCommand(double speed) {
        return this.startEnd(() -> m_Lift.set(speed), () -> m_LiftPID.setReference(m_LiftEncoder.getPosition(), CANSparkMax.ControlType.kPosition));
        //return this.startEnd(() ->  m_Lift.set(speed), () -> m_Lift.set(Constants.RotatingArmConstants.liftHoldSpeed));

    }

    public CommandBase rotateWristCommand(double speed){
      
        //return this.startEnd(() -> m_Wrist.set(speed), () -> m_WristPID.setReference(m_WristEncoder.getPosition(), CANSparkMax.ControlType.kPosition));
        return this.startEnd(() -> m_Wrist.set(speed), () -> m_Wrist.set(Constants.RotatingArmConstants.wristHoldSpeed));
    }

    public CommandBase wristSetReference(){
        final double pos = (m_WristEncoder.getPosition() < 5) ? 5 : m_WristEncoder.getPosition();
        return this.runOnce(() -> m_WristPID.setReference(pos, CANSparkMax.ControlType.kPosition));
    }

    public void setArmPosition(double armAngle, double wristAngle){
        m_LiftPID.setReference(armAngle, CANSparkMax.ControlType.kPosition);
        m_WristPID.setReference(wristAngle, CANSparkMax.ControlType.kPosition);
    }

    public Command setArmPositionCommand(double armAngle, double wristAngle){
        return this.runOnce(() -> setArmPosition(armAngle, wristAngle));
    }

    //TODO check wrist angles
    public CommandBase armTopGoal() {
        return this.runOnce( () -> setArmPosition(200, 90));
    }
    public CommandBase armMidGoal() {
        return this.runOnce( () -> setArmPosition(150, 70));
    }
    public CommandBase armLowGoal() {
        return this.runOnce( () -> setArmPosition(30, 60));
    }
    public CommandBase armFloor() {
        return this.runOnce( () -> setArmPosition(30, 60));
    }



}
