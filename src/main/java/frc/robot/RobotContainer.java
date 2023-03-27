// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Joystick;
//import edu.wpi.first.wpilibj.Joystick;
//import edu.wpi.first.wpilibj.XboxController;
//import edu.wpi.first.wpilibj.XboxController.Button;
//import edu.wpi.first.wpilibj.PS4Controller.Button;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
//import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.List;

import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.cameraserver.CameraServer;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
        // The robot's subsystems
        public final static WPI_Pigeon2 m_gyro = new WPI_Pigeon2(0);
        // The driver's controller
        // public final static XboxController m_driverController = new
        // XboxController(OIConstants.kDriverControllerPort);
        // public final static XboxController m_gunnerController = new
        // XboxController(OIConstants.kGunnerControllerPort);
        public final static Joystick m_driverController = new Joystick(0);
        public final static Joystick m_gunnerController = new Joystick(1);
        public final static DriveSubsystem m_robotDrive = new DriveSubsystem();
        public final static RotatingArm m_robotArm = new RotatingArm();
        public final static Intake m_robotIntake = new Intake();

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                // Configure the button bindings
                configureButtonBindings();
                // configureDashboard();
                CameraServer.startAutomaticCapture();

                // Configure default commands
                if (Constants.DriveConstants.driveRightJoystick) {
                        m_robotDrive.setDefaultCommand(m_robotDrive.driveRightJoystick());
                } else {
                        m_robotDrive.setDefaultCommand(m_robotDrive.driveLeftJoystick());
                }

                //RotatingArm.m_LiftPID.setReference(5, CANSparkMax.ControlType.kPosition);
                //RotatingArm.m_WristPID.setReference(2, CANSparkMax.ControlType.kPosition);

        }

        // Set up initial Shuffleboard boxes, not sure if neccesary
        // private void configureDashboard() {
        // SmartDashboard.putData("Wrist Encoder Position", null);
        // SmartDashboard.putData("Lift Encoder Position", null);
        // SmartDashboard.putData("Gyro", null);
        // }

        /**
         * Use this method to define your button->command mappings. Buttons can be
         * created by
         * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
         * subclasses ({@link
         * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
         * passing it to a
         * {@link JoystickButton}.
         */
        private void configureButtonBindings() {
                new JoystickButton(m_driverController, 1)
                                .whileTrue(m_robotDrive.setXCommand());

                // Rotate arm up
                new JoystickButton(m_gunnerController, 4)
                                .whileTrue(m_robotArm.rotateLiftCommand(Constants.RotatingArmConstants.liftMaxSpeed));

                // Rotate arm down
                new JoystickButton(m_gunnerController, 1)
                                .whileTrue(m_robotArm
                                                .rotateLiftCommand(Constants.RotatingArmConstants.liftMaxSpeed * -.5));
                // Rotate Wrist up
                new JoystickButton(m_gunnerController, 3)
                                .whileTrue(m_robotArm.rotateWristCommand(Constants.RotatingArmConstants.wristMaxSpeed));
                // Rotate Wrist down
                new JoystickButton(m_gunnerController, 2)
                                .whileTrue(m_robotArm
                                                .rotateWristCommand(-Constants.RotatingArmConstants.wristMaxSpeed));
                // Intake on claw
                new JoystickButton(m_gunnerController, 5)
                                .whileTrue(m_robotIntake.outake());
                // Outake on claw
                new JoystickButton(m_gunnerController, 6)
                                .whileTrue(m_robotIntake.intake());
                /*
                 * top -> high goal
                 * left -> mid goal
                 * bottom -> low goal
                 * right -> floor
                 */
                POVButton povUP = new POVButton(m_gunnerController, 0);
                povUP.onTrue(m_robotArm.armTopGoal());
                POVButton povLeft = new POVButton(m_gunnerController, 90);
                povLeft.onTrue(m_robotArm.armMidGoal());
                POVButton povDown = new POVButton(m_gunnerController, 180);
                povDown.onTrue(m_robotArm.armLowGoal());
                POVButton povRight = new POVButton(m_gunnerController, 270);
                povRight.onTrue(m_robotArm.armFloor());

        }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {
                // Create config for trajectory
                TrajectoryConfig config = new TrajectoryConfig(
                                AutoConstants.kMaxSpeedMetersPerSecond,
                                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                                // Add kinematics to ensure max speed is actually obeyed
                                .setKinematics(DriveConstants.kDriveKinematics);

                // An example trajectory to follow. All units in meters.
                Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
                                // Start at the origin facing the +X direction
                                new Pose2d(0, 0, new Rotation2d(0)),
                                // Pass through these two interior waypoints, making an 's' curve path
                                List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
                                // End 3 meters straight ahead of where we started, facing forward
                                new Pose2d(3, 0, new Rotation2d(0)),
                                config);


                var thetaController = new ProfiledPIDController(
                                AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
                thetaController.enableContinuousInput(-Math.PI, Math.PI);

                SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
                                exampleTrajectory,
                                m_robotDrive::getPose, // Functional interface to feed supplier
                                DriveConstants.kDriveKinematics,

                                // Position controllers
                                new PIDController(AutoConstants.kPXController, 0, 0),
                                new PIDController(AutoConstants.kPYController, 0, 0),
                                thetaController,
                                m_robotDrive::setModuleStates,
                                m_robotDrive);

                // Reset odometry to the starting pose of the trajectory.
                m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

                // Run path following command, then stop at the end.
                return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false));
        }

}
