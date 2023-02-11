// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
// import frc.robot.commands.Autos;
// import frc.robot.commands.ExampleCommand;
// import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AutoAline;
import frc.robot.commands.autoAlineYaw;
import frc.robot.commands.infoFromLibrary;
import frc.robot.subsystems.drivetrain;
// import frc.robot.commands.tagInfo;
// import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.photonvision2;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  // private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  // public final PhotonVision p_photonVision = new PhotonVision();
  public final static photonvision2 p_photonvision2 = new photonvision2();
  public final static drivetrain d_drivetrain = new drivetrain();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  // private final CommandXboxController m_driverController =
  //     new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */

  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    // p_photonVision.setDefaultCommand(new tagInfo(p_photonVision));
    // p_photonvision2.setDefaultCommand(new infoFromLibrary(p_photonvision2));
    // d_drivetrain.setDefaultCommand(new AutoAline());
    // d_drivetrain.setDefaultCommand(new autoAlineYaw());
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or 
   * via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  public static Joystick Xcontrol = new Joystick(0);

  public static JoystickButton AB = new JoystickButton(Xcontrol, 1);
  public static JoystickButton BB = new JoystickButton(Xcontrol, 2);

  private void configureBindings() {
    AB.whileTrue(new AutoAline());
    // BB.onTrue(new autoAlineYaw());
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    // new Trigger(m_exampleSubsystem::exampleCondition)
    //     .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    // m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new InstantCommand();
    // An example command will be run in autonomous
  }

  public Command getInitCommand(){
    // return new tagInfo(p_photonVision);
    return new InstantCommand();
    // return new infoFromLibrary(p_photonvision2);

  }
}
