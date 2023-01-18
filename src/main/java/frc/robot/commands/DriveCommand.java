// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class DriveCommand extends CommandBase {
  private final DriveSubsystem drivetrain;
  private final XboxController ctr;
  private final Boolean fieldOrientated;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DriveCommand(XboxController XBox, DriveSubsystem drivetrain, Boolean fieldOrientated) {
    this.drivetrain = drivetrain;
    this.ctr = XBox;
    this.fieldOrientated = fieldOrientated;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Constants.DriveConstants.driveRightJoystick) {
      drivetrain.drive(ctr.getRightX(), -ctr.getRightY(), ctr.getLeftX(),
          fieldOrientated);
    } else {
      drivetrain.drive(ctr.getLeftX(), -ctr.getLeftY(), ctr.getRightX(),
          fieldOrientated);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false; // always false bc its driver program
  }
}
