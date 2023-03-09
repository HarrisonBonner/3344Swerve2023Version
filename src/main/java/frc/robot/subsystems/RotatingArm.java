package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
//import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
//import com.revrobotics.SparkMaxPIDController;
//import com.revrobotics.CANSparkMax.ControlType;
//import com.revrobotics.AbsoluteEncoder;
//import com.revrobotics.RelativeEncoder;

import frc.robot.Constants;


public class RotatingArm extends SubsystemBase {

    private final CANSparkMax m_Lift = new CANSparkMax(Constants.RotatingArmConstants.rotationMotorCanID,
            MotorType.kBrushless);
    private final CANSparkMax m_Wrist = new CANSparkMax(Constants.RotatingArmConstants.wristMotorCanId,
            MotorType.kBrushless);

   
    public RotatingArm() {
        m_Lift.setIdleMode(CANSparkMax.IdleMode.kBrake);
        m_Wrist.setIdleMode(CANSparkMax.IdleMode.kBrake);
        m_Wrist.burnFlash();
        m_Lift.burnFlash();

    }

    public void periodic() {
    }

    public void stopRotationMotors(){
        m_Lift.stopMotor();
    }

    public CommandBase rotateCommand(double speed) {

        return this.startEnd(() ->  rotate(speed), () ->  stopRotationMotors());

    }

    public void rotate(double speed){
        m_Lift.set(speed);
    }

   



}
