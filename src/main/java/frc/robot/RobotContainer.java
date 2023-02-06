// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.RamseteAutoBuilder;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.BeltControl;
import frc.robot.commands.ShooterControl;
import frc.robot.commands.Drivetrain.ChangeSpeed;
import frc.robot.commands.Drivetrain.TankDrive;
import frc.robot.commands.Drivetrain.ToggleDirection;
import frc.robot.commands.IntakeCmds.IntakeMotorControl;
// import frc.robot.commands.IntakeCmds.SolControl;
import frc.robot.commands.Utils.Time;
import frc.robot.subsystems.Belt;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.AutoPaths.LoadPath;
import frc.robot.subsystems.Intake.AirSys;
import frc.robot.subsystems.Intake.MotorSys;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  public final static DriveTrain m_drivetrain = new DriveTrain();
  public final static Shooter m_shooter = new Shooter(2);
  public final static Belt m_belt = new Belt(0);
  public final static Elevator m_elevator = new Elevator();
  // public final static AirSys m_airsys = new AirSys();
  public final static MotorSys m_motorsys= new MotorSys();

  // Controller
  public static Joystick XboxC = new Joystick(0);
  public static Joystick buttonBox = new Joystick(1); // ButtonBox that controls speed and direction

  // Controller Buttons
  public static JoystickButton YB = new JoystickButton(XboxC, 4);
  public static JoystickButton AB = new JoystickButton(XboxC, 1);
  public static JoystickButton XB = new JoystickButton(XboxC, 3);
  public static JoystickButton BB = new JoystickButton(XboxC, 2);

  public static JoystickButton LBB = new JoystickButton(XboxC, 5);
  public static JoystickButton RBB = new JoystickButton(XboxC, 6);

  // D-pad
  public static POVButton upButton = new POVButton(XboxC, 0);
  public static POVButton downButton = new POVButton(XboxC, 180);

  // Buttons from the buttonbox
  public static JoystickButton B1 = new JoystickButton(buttonBox, 11);
  public static JoystickButton B2 = new JoystickButton(buttonBox, 10);
  public static JoystickButton B3 = new JoystickButton(buttonBox, 9);
  public static JoystickButton B4 = new JoystickButton(buttonBox, 8);
  
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  // PathPlanner AutoBuilder, builds a full autonomous command
  RamseteAutoBuilder testRouteBuilder = new RamseteAutoBuilder(
    m_drivetrain::getPose,
    m_drivetrain::resetOdometery,
    new RamseteController(Constants.AutoConstants.kRamseteB, Constants.AutoConstants.kRamseteZeta),
    Constants.DriveConstants.kDriveKinematics,
    new SimpleMotorFeedforward(
      Constants.AutoConstants.ksVolts,
      Constants.AutoConstants.kaVoltSecondsSquaredPerMeter,
      Constants.AutoConstants.kvVoltSecondsPerMeter
    ),
    m_drivetrain::getWheelSpeeds,
    new PIDConstants(Constants.AutoConstants.kPDriveVel, 0, 0),
    m_drivetrain::tankDriveVolts,
    Constants.AutoConstants.testEventMap,
    m_drivetrain
  );

  public static LoadPath m_pathTest = new LoadPath("test", true);
  private final Command testAuto = testRouteBuilder.fullAuto(LoadPath.pathToFollow);
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    m_drivetrain.setDefaultCommand(new TankDrive());

    configureButtonBindings();
    Shuffleboard.getTab("Auto").add(m_chooser);
    m_chooser.addOption("Test", new SequentialCommandGroup(
      m_drivetrain.showTraj(0),
      testAuto
    ));
    m_chooser.addOption("Nothing", new InstantCommand());

    // Fill in HashMap values
    setTestEventMap();
    System.out.println(Constants.AutoConstants.testEventMap);
  }

  public static double getDeadZone(int axis) {
    double rawAxisValue = Double.valueOf(XboxC.getRawAxis(axis));
    if (rawAxisValue >= 0.08 || rawAxisValue <= -0.08) {
      Math.round(rawAxisValue * 100 / 100);
    } else {
      rawAxisValue = 0.0;
    }

    return -rawAxisValue;
  }
  

  public void setTestEventMap() {
    Constants.AutoConstants.testEventMap.put("Start", new InstantCommand());
    Constants.AutoConstants.testEventMap.put("Stop", new SequentialCommandGroup(
      new ShooterControl(2),
      new ParallelCommandGroup(new ShooterControl(false), new BeltControl(false)),
      new ParallelCommandGroup(new BeltControl(true), new ShooterControl(true)),
      new Time(1000),
      new ParallelCommandGroup(new ShooterControl(false), new BeltControl(false))

    ));
  }


  /**
   * Use this method to define your button->command mappings. Buttons can be created by 
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    B1.onTrue(new ChangeSpeed(1));
    B2.onTrue(new ChangeSpeed(0.75));
    B3.onTrue(new ChangeSpeed(0.5));

    B4.onTrue(new ToggleDirection());

    RBB.onTrue(new ShooterControl(true));
    RBB.onFalse(new ShooterControl(false));

    RBB.onTrue(new BeltControl(true));
    RBB.onFalse(new BeltControl(false));

    RBB.onTrue(new IntakeMotorControl(0.75));
    RBB.onFalse(new IntakeMotorControl(0));

    AB.onTrue(new ShooterControl(0));
    AB.onTrue(new BeltControl(0));
    XB.onTrue(new ShooterControl(1));
    YB.onTrue(new ShooterControl(2));
    BB.onTrue(new ShooterControl(3));
    BB.onTrue(new BeltControl(1));

    // upButton.onTrue(new SolControl(true));
    // downButton.onTrue(new SolControl(false));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // return m_chooser.getSelected();
    return new InstantCommand();
  }
}