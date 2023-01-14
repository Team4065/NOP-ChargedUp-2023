// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  public final static Elevator m_elevator = new Elevator();
  public final static Drivetrain m_drivetrain = new Drivetrain();


  // Replace with CommandPS4Controller or CommandJoystick if needed
  private static Joystick JoyC =  new Joystick(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */

   public static JoystickButton AB = new JoystickButton(JoyC, 1);
  public static JoystickButton BB = new JoystickButton(JoyC, 2);
  public static JoystickButton XB = new JoystickButton(JoyC, 3);
  public static JoystickButton YB = new JoystickButton(JoyC, 4);
  public static JoystickButton LB = new JoystickButton(JoyC, 5);
  public static JoystickButton RB = new JoystickButton(JoyC, 6);
  public static JoystickButton HB = new JoystickButton(JoyC, 7);
  public static JoystickButton ZB = new JoystickButton(JoyC, 8);
  public static JoystickButton LJB = new JoystickButton(JoyC, 9);
  public static JoystickButton RJB = new JoystickButton(JoyC, 10);

  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }
  public static double getDeadZone(int axis){
    return axis;
  }

  public static double getAxisRamped(int axis){
    return 1.0377241992 * (2 / (1 + Math.pow(Math.E, -4 * JoyC.getRawAxis(axis))));
  }

}
