// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.Swing;
import frc.robot.commands.Drivetrain.ArcadeDrive;
import frc.robot.commands.Drivetrain.TankDrive;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.SwingArm;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  public final static DriveTrain m_drivetrain = new DriveTrain();
  public final static Claw m_claw = new Claw();
  public final static SwingArm m_swingarm = new SwingArm();


  // Controller
  public static Joystick XboxC = new Joystick(0);

  // Controller Joysticks

  // Controller Buttons
  public static JoystickButton YB = new JoystickButton(XboxC, 4);
  public static JoystickButton AB = new JoystickButton(XboxC, 1);
  public static JoystickButton XB = new JoystickButton(XboxC, 3);
  public static JoystickButton BB = new JoystickButton(XboxC, 2);

  // D-pad
  public static POVButton upButton = new POVButton(XboxC, 0);

  SendableChooser<Command> m_chooser = SendableChooser<>();
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    m_drivetrain.setDefaultCommand(new TankDrive());
    m_swingarm.setDefaultCommand(new Swing());

    configureButtonBindings();

    m_chooser.addOption("Straight", getAutonomousCommand());
  }

  public Command loadPathToRam(String fileName, boolean resetOdo) {
    Trajectory trajectory;
    try {
      Path trajecPath = Filesystem.getDeployDirectory().toPath().resolve(fileName);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajecPath);

    } catch (IOException exception) {
      // Send errors
      DriverStation.reportError("Unable to open " + fileName, exception.getStackTrace());
      System.out.println("Unable to open " + fileName);
      // Set autonomous command to a blank command
      return new InstantCommand();
    }
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // YB.whenPressed(new FlagCmd(true, m_pne));
    BB.onTrue(new TankDrive());
    XB.onTrue(new ArcadeDrive());    
    // YB.onTrue(new ClawOpenAndClose(200, 0.1, 0.9));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new InstantCommand();
  }
}
