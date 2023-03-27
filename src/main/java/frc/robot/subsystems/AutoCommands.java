package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import java.util.List;
import frc.robot.RobotContainer;

public class AutoCommands {

    public AutoCommands() {
    }

    public Command moveAndScoreAuto() {
        TrajectoryConfig config = new TrajectoryConfig(
                Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                // Add kinematics to ensure max speed is actually obeyed
                .setKinematics(Constants.DriveConstants.kDriveKinematics);

        config.setReversed(true);

        Trajectory firstMove = TrajectoryGenerator.generateTrajectory(new Pose2d(5, 0, new Rotation2d(0)),
                List.of(new Translation2d(.5, 0)), new Pose2d(0, 0, new Rotation2d(0)), config);

        // Trajectory finalMove = TrajectoryGenerator.generateTrajectory(List.of(new
        // Pose2d(0, 0, new Rotation2d(0))),
        // config);

        var thetaController = new ProfiledPIDController(
                Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand firstMoveSwerveControllerCommand = new SwerveControllerCommand(
                firstMove,
                RobotContainer.m_robotDrive::getPose, // Functional interface to feed supplier
                Constants.DriveConstants.kDriveKinematics,

                // Position controllers
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                RobotContainer.m_robotDrive::setModuleStates,
                RobotContainer.m_robotDrive);

        // SwerveControllerCommand finalMoveSwerveControllerCommand = new
        // SwerveControllerCommand(
        // finalMove,
        // RobotContainer.m_robotDrive::getPose, // Functional interface to feed
        // supplier
        // Constants.DriveConstants.kDriveKinematics,

        // // Position controllers
        // new PIDController(Constants.AutoConstants.kPXController, 0, 0),
        // new PIDController(Constants.AutoConstants.kPYController, 0, 0),
        // thetaController,
        // RobotContainer.m_robotDrive::setModuleStates,
        // RobotContainer.m_robotDrive);

        // Reset odometry to the starting pose of the trajectory.
        RobotContainer.m_robotDrive.resetOdometry(firstMove.getInitialPose());

        // Run path following command, then stop at the end.
        return RobotContainer.m_robotArm.setArmPositionCommand(200, 92)
                .andThen(new WaitCommand(1.5))
                .andThen(RobotContainer.m_robotIntake.outakeConeCommand())
                .andThen(new WaitCommand(1))
                .andThen(RobotContainer.m_robotIntake.stopIntake())
                // .andThen(RobotContainer.m_robotArm.setArmPositionCommand(20, 20))
                .andThen(firstMoveSwerveControllerCommand)
                .andThen(() -> RobotContainer.m_robotDrive.drive(0, 0, 0, false))
                .andThen(RobotContainer.m_robotArm.setArmPositionCommand(20, 20));
    }

    public Command sCurve() {
        // Create config for trajectory
        TrajectoryConfig config = new TrajectoryConfig(
                Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                // Add kinematics to ensure max speed is actually obeyed
                .setKinematics(Constants.DriveConstants.kDriveKinematics);

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
                Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
                exampleTrajectory,
                RobotContainer.m_robotDrive::getPose, // Functional interface to feed supplier
                Constants.DriveConstants.kDriveKinematics,

                // Position controllers
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                RobotContainer.m_robotDrive::setModuleStates,
                RobotContainer.m_robotDrive);

        // Reset odometry to the starting pose of the trajectory.
        RobotContainer.m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

        // Run path following command, then stop at the end.
        return swerveControllerCommand.andThen(() -> RobotContainer.m_robotDrive.drive(0, 0, 0, false));
    }

    public Command moveAndScoreAndLevelAuto() {
        TrajectoryConfig config = new TrajectoryConfig(
                Constants.AutoConstants.kMaxSpeedMetersPerSecond / 2,
                Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared / 2)
                // Add kinematics to ensure max speed is actually obeyed
                .setKinematics(Constants.DriveConstants.kDriveKinematics);

        config.setReversed(true);

        Trajectory firstMove = TrajectoryGenerator.generateTrajectory(new Pose2d(5, 0, new Rotation2d(0)),
                List.of(new Translation2d(4.5, 0)), new Pose2d(4.2, 0, new Rotation2d(0)), config);

        Trajectory secondMove = TrajectoryGenerator.generateTrajectory(new Pose2d(4, 0, new Rotation2d(0)),
                List.of(new Translation2d(3.7, 0)), new Pose2d(3.5, 0, new Rotation2d(0)), config);
        // Trajectory finalMove = TrajectoryGenerator.generateTrajectory(List.of(new
        // Pose2d(0, 0, new Rotation2d(0))),
        // config);

        // config.setReversed(false);

        // Trajectory thirdMove = TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, new Rotation2d(0)),
        //         List.of(new Translation2d(.5, 0)), new Pose2d(1.5, 0, new Rotation2d(0)), config);

        var thetaController = new ProfiledPIDController(
                Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand firstMoveSwerveControllerCommand = new SwerveControllerCommand(
                firstMove,
                RobotContainer.m_robotDrive::getPose, // Functional interface to feed supplier
                Constants.DriveConstants.kDriveKinematics,

                // Position controllers
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                RobotContainer.m_robotDrive::setModuleStates,
                RobotContainer.m_robotDrive);

        SwerveControllerCommand secondMoveSwerveControllerCommand = new SwerveControllerCommand(
                secondMove,
                RobotContainer.m_robotDrive::getPose, // Functional interface to feed supplier
                Constants.DriveConstants.kDriveKinematics,

                // Position controllers
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                RobotContainer.m_robotDrive::setModuleStates,
                RobotContainer.m_robotDrive);

        // SwerveControllerCommand thirdMoveSwerveControllerCommand = new SwerveControllerCommand(
        //         thirdMove,
        //         RobotContainer.m_robotDrive::getPose, // Functional interface to feed supplier
        //         Constants.DriveConstants.kDriveKinematics,

        //         // Position controllers
        //         new PIDController(Constants.AutoConstants.kPXController, 0, 0),
        //         new PIDController(Constants.AutoConstants.kPYController, 0, 0),
        //         thetaController,
        //         RobotContainer.m_robotDrive::setModuleStates,
        //         RobotContainer.m_robotDrive);
       

        // Reset odometry to the starting pose of the trajectory.
        RobotContainer.m_robotDrive.resetOdometry(firstMove.getInitialPose());

        // Run path following command, then stop at the end.
        return RobotContainer.m_robotArm.setArmPositionCommand(200, 92)
                .andThen(new WaitCommand(1.5))
                .andThen(RobotContainer.m_robotIntake.outakeCubeCommand())
                .andThen(new WaitCommand(1))
                .andThen(RobotContainer.m_robotIntake.stopIntake())
                .andThen(firstMoveSwerveControllerCommand)
                .andThen(RobotContainer.m_robotArm.setArmPositionCommand(20, 20))
                .andThen(secondMoveSwerveControllerCommand)
                //.andThen(thirdMoveSwerveControllerCommand)
                .andThen(RobotContainer.m_robotDrive.autoLevelCommand())
                .andThen(() -> RobotContainer.m_robotDrive.drive(0, 0, 0, false));

    }
}
