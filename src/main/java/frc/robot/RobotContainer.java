// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.Drivetrain.ChangeSpeed;
import frc.robot.commands.Drivetrain.TankDrive;
import frc.robot.subsystems.DriveTrain;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  public final static DriveTrain m_drivetrain = new DriveTrain();


  // Controller
  public static Joystick XboxC = new Joystick(0);
  public static Joystick buttonBox = new Joystick(1); // ButtonBox that controls speed and direction

  // Controller Buttons
  public static JoystickButton YB = new JoystickButton(XboxC, 4);
  public static JoystickButton AB = new JoystickButton(XboxC, 1);
  public static JoystickButton XB = new JoystickButton(XboxC, 3);
  public static JoystickButton BB = new JoystickButton(XboxC, 2);

  // D-pad
  public static POVButton upButton = new POVButton(XboxC, 0);

  // Buttons from the buttonbox
  public static JoystickButton B1 = new JoystickButton(buttonBox, 11);
  public static JoystickButton B2 = new JoystickButton(buttonBox, 10);
  public static JoystickButton B3 = new JoystickButton(buttonBox, 9);
  public static JoystickButton B4 = new JoystickButton(buttonBox, 8);
  
  SendableChooser<Command> m_chooser = new SendableChooser<>();
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    m_drivetrain.setDefaultCommand(new TankDrive());
    // m_swingarm.setDefaultCommand(new Swing());

    configureButtonBindings();
    m_chooser.addOption("Test", m_drivetrain.loadPathRam("deploy/pathplanner/generatedJSON/test.wpilib.json", true));
    Shuffleboard.getTab("Auto").add(m_chooser);
  }

  public static double getDeadZone(int axis){
    return -XboxC.getRawAxis(axis);
  }


  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    B1.onTrue(new ChangeSpeed(1));
    B2.onTrue(new ChangeSpeed(0.5));
    B3.onTrue(new ChangeSpeed(0.2));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }
}
