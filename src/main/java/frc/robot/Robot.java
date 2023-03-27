// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.AutoCommands;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  private AutoCommands m_AutoCommands;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  private static final String kMoveAndScoreAuto = "Move and Score";
  private static final String kScoreAndLevelAuto = "Score and Level";
  private static final String kScoreCube = "Score Cube Only";
  private static final String kScoreCone = "Score Cone Only";

  private static String autoSelected;

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    m_AutoCommands = new AutoCommands();

    m_chooser.addOption("Move and Score", kMoveAndScoreAuto);
    m_chooser.addOption("Score and Level", kScoreAndLevelAuto);
    m_chooser.addOption("Score Cone Only", kScoreCone);
    m_chooser.addOption("Score Cube Only", kScoreCone);
    SmartDashboard.putData("Auto choices", m_chooser);

  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items
   * like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    // m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    // m_autonomousCommand = m_AutoCommands.moveAndScoreAuto();
    // m_autonomousCommand = m_AutoCommands.sCurve();
    // m_autonomousCommand = m_AutoCommands.moveAndScoreAndLevelAuto();

    //Choose auto from shuffleboard
    autoSelected = m_chooser.getSelected();

    switch (autoSelected) {
      case kMoveAndScoreAuto:
        // Put custom auto code here
        m_autonomousCommand = m_AutoCommands.moveAndScoreAuto();
        break;
      case kScoreAndLevelAuto:
        m_autonomousCommand = m_AutoCommands.moveAndScoreAndLevelAuto();
        break;
      case kScoreCone:
        m_autonomousCommand = m_AutoCommands.justScoreCone();
        break;
      case kScoreCube:
        m_autonomousCommand = m_AutoCommands.justScoreCube();
      default:
        // Put default auto code here
        m_autonomousCommand = null;
        break;
    }

    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector",
     * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
     * = new MyAutoCommand(); break; case "Default Auto": default:
     * autonomousCommand = new ExampleCommand(); break; }
     */

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }
}
