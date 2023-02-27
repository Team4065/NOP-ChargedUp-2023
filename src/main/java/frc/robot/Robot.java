// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

<<<<<<< Updated upstream
=======
import java.util.Map;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
>>>>>>> Stashed changes
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.Drivetrain.AutoBalance;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

<<<<<<< Updated upstream
=======
  public static final ShuffleboardTab mainTab = Shuffleboard.getTab("VALUES");


  ShuffleboardTab autoTab = Shuffleboard.getTab("Auto");
  GenericEntry allianceColor = 
    autoTab.add("ALLIANCE", true) 
      .withProperties(Map.of("colorWhenTrue", "blue"))
      .withPosition(0, 1)
      .withSize(3, 1)
      .getEntry();

>>>>>>> Stashed changes
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
<<<<<<< Updated upstream
    // autonomous chooser on the dashboard.
=======
    // autonomous chooser on the dashboard
>>>>>>> Stashed changes
    m_robotContainer = new RobotContainer();
    CameraServer.startAutomaticCapture();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
<<<<<<< Updated upstream
  public void disabledInit() {}
=======
  public void disabledInit() {
    // TO-DO: REMOVE THIS LINE AND CHANGE IT TO BRAKE MODE
    AutoBalance.stop = true;
  }
>>>>>>> Stashed changes

  @Override
  public void disabledPeriodic() {
    if (RobotContainer.autoMap.get(RobotContainer.m_chooser.getSelected()) != "nothing") {
      RobotContainer.m_drivetrain.showTraj(RobotContainer.autoMap.get(RobotContainer.m_chooser.getSelected()));
    } else {
      RobotContainer.m_drivetrain.showTraj();
    }

    if (DriverStation.getAlliance() == DriverStation.Alliance.Blue) {
      allianceColor.setBoolean(true);
    } else if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
      allianceColor.setBoolean(false);
    } else {
      allianceColor.setString("ERROR");
    }
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
<<<<<<< Updated upstream
  public void autonomousInit() {
=======
  public void autonomousInit() { 
    RobotContainer.m_drivetrain.setBreakMode();
    RobotContainer.m_drivetrain.resetEncoders();
    RobotContainer.m_drivetrain.zeroHeading();
    RobotContainer.m_drivetrain.resetOdometery(new Pose2d(0, 0, new Rotation2d()));
>>>>>>> Stashed changes
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      System.out.println("reached");
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

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
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
