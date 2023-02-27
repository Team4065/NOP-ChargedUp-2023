// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

<<<<<<< Updated upstream
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
=======
import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.RamseteAutoBuilder;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
>>>>>>> Stashed changes
import edu.wpi.first.wpilibj2.command.Command;
<<<<<<< Updated upstream
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
=======
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
<<<<<<< Updated upstream
import edu.wpi.first.wpilibj2.command.PrintCommand;
=======
>>>>>>> Stashed changes
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.BeltControl;
import frc.robot.commands.ChangeLED;
import frc.robot.commands.ShooterControl;
<<<<<<< Updated upstream
=======
import frc.robot.commands.Drivetrain.AutoAlign;
import frc.robot.commands.Drivetrain.AutoBalance;
>>>>>>> Stashed changes
import frc.robot.commands.Drivetrain.ChangeSpeed;
import frc.robot.commands.Drivetrain.TankDrive;
import frc.robot.commands.Drivetrain.ToggleDirection;
import frc.robot.commands.IntakeCmds.IntakeMotorControl;
import frc.robot.commands.IntakeCmds.SolControl;
import frc.robot.commands.Utils.Time;
import frc.robot.subsystems.Belt;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.RaspPiCam;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Intake.AirSys;
import frc.robot.subsystems.Intake.MotorSys;
>>>>>>> Stashed changes

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
<<<<<<< Updated upstream
<<<<<<< Updated upstream
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
=======
=======

>>>>>>> Stashed changes
  public final static DriveTrain m_drivetrain = new DriveTrain();
  public final static Shooter m_shooter = new Shooter(2);
  public final static Belt m_belt = new Belt(0);
  public final static Elevator m_elevator = new Elevator();
  public final static AirSys m_airsys = new AirSys();
  public final static MotorSys m_motorsys = new MotorSys();
  public final static LEDs m_leds = new LEDs();
  public final static RaspPiCam m_rasppicam = new RaspPiCam();

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
  public static POVButton upbutton = new POVButton(XboxC, 0);
  public static POVButton downbutton = new POVButton(XboxC, 180);
  

  // Buttons from the buttonbox
  public static JoystickButton B1 = new JoystickButton(buttonBox, 11);
  public static JoystickButton B2 = new JoystickButton(buttonBox, 10);
  public static JoystickButton B3 = new JoystickButton(buttonBox, 9);
  public static JoystickButton B4 = new JoystickButton(buttonBox, 8);
  public static JoystickButton B5 = new JoystickButton(buttonBox, 7);
  public static JoystickButton B6 = new JoystickButton(buttonBox, 6);
  public static JoystickButton B7 = new JoystickButton(buttonBox, 12);
<<<<<<< Updated upstream
  
  public static SendableChooser<Command> m_chooser = new SendableChooser<>();

  public Command ramAutoBuilder(String pathName, HashMap<String, Command> eventMap) {
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
      eventMap,
      m_drivetrain
    );

    List<PathPlannerTrajectory> pathToFollow = PathPlanner.loadPathGroup(pathName, PathPlanner.getConstraintsFromPath(pathName));
    final Command auto = testRouteBuilder.fullAuto(pathToFollow);
    return auto;
  }

  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Fill in the HashMap values
    setTestEventMap();
    setAnotherTestEventMap();
    m_drivetrain.setDefaultCommand(new TankDrive());

    configureButtonBindings(); //Configure the button bindings
    Shuffleboard.getTab("Auto").add(m_chooser);
    m_chooser.setDefaultOption("Nothing", new InstantCommand());
    m_chooser.addOption("Test", ramAutoBuilder("test", Constants.AutoConstants.testEventMap));
    m_chooser.addOption("Another Test", ramAutoBuilder("anotherTest", Constants.AutoConstants.anotherTestEventMap));
=======

  // Other
  public static HashMap<Command, String> autoMap = new HashMap<>();

  public Command ramAutoBuilder(String pathName, HashMap<String, Command> eventMap) {
    // PathPlanner AutoBuilder, builds a full autonomous command
    RamseteAutoBuilder pathBuilder = new RamseteAutoBuilder(
        m_drivetrain::getPose,
        m_drivetrain::resetOdometery,
        new RamseteController(Constants.AutoConstants.kRamseteB, Constants.AutoConstants.kRamseteZeta),
        Constants.DriveConstants.kDriveKinematics,
        new SimpleMotorFeedforward(
            Constants.AutoConstants.ksVolts,
            Constants.AutoConstants.kvVoltSecondsPerMeter,
            Constants.AutoConstants.kaVoltSecondsSquaredPerMeter),
        m_drivetrain::getWheelSpeeds,
        new PIDConstants(Constants.AutoConstants.kPDriveVel, 0, 0),
        m_drivetrain::tankDriveVolts,
        eventMap,
        true,
        m_drivetrain);
    List<PathPlannerTrajectory> pathToFollow = PathPlanner.loadPathGroup(pathName,
        PathPlanner.getConstraintsFromPath(pathName));
    final Command auto = pathBuilder.fullAuto(pathToFollow);
    autoMap.put(auto, pathName);
    return auto;
  }

  public static SendableChooser<Command> m_chooser = new SendableChooser<>();
  public static HashMap<Command, String> autoRoutines = new HashMap<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Fill in the HashMap values
    RobotContainer.m_drivetrain.zeroHeading();
    RobotContainer.m_drivetrain.resetEncoders();

    setRed3GPMap();
    setMidMap();
    setRed3GPABMap();
    setRed1GPEventMap();
    setRed1GPABEventMap();

    m_drivetrain.setDefaultCommand(new TankDrive());

    configureButtonBindings(); // Configure the button bindings
    Shuffleboard.getTab("Auto").add(m_chooser).withSize(3, 1);
    Command instantCmd = new InstantCommand();
    m_chooser.setDefaultOption("Nothing", instantCmd);
    autoMap.put(instantCmd, "nothing");
    m_chooser.addOption("Pos 1 - 2 Game Pieces", ramAutoBuilder("Red 1 - 2 GP", Constants.AutoConstants.red1GPEventMap));
    m_chooser.addOption("Pos 1 - 2 Game Pieces & Auto Balance", ramAutoBuilder("Red 1 - 2 GP AB", Constants.AutoConstants.red1GPABEventMap));
    m_chooser.addOption("Pos 2 - Preload & Auto Balance", ramAutoBuilder("Mid", Constants.AutoConstants.mid));
    m_chooser.addOption("Pos 3 - 2 Game Pieces", ramAutoBuilder("Red 3 - 2 GP", Constants.AutoConstants.red3GPEventMap));
    m_chooser.addOption("Pos 3 - Preload & Auto Balance", ramAutoBuilder("Red 3 - 1 GP AB", Constants.AutoConstants.red3GPABEventMap));
    
    m_chooser.addOption("Balance Test", new SequentialCommandGroup(
      new GetOnRamp(),
      new AutoBalance()
    ));
>>>>>>> Stashed changes
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
      new ParallelCommandGroup(new ShooterControl(true), new BeltControl(true, false)),
      new Time(1000),
      new ParallelCommandGroup(new ShooterControl(false), new BeltControl(false, false))
    ));
  }

  public void setAnotherTestEventMap() {
    Constants.AutoConstants.anotherTestEventMap.put("Start", new PrintCommand("Start"));
    Constants.AutoConstants.anotherTestEventMap.put("Stop", new SequentialCommandGroup(
      new ParallelCommandGroup(new ShooterControl(true), new BeltControl(true, false)),
      new Time(1000),
      new ParallelCommandGroup(new ShooterControl(false), new BeltControl(false, false))
    ));
>>>>>>> Stashed changes
  }

  public void setRed3GPABMap() {
    Constants.AutoConstants.red3GPABEventMap.put("Start", new SequentialCommandGroup(
        new SequentialCommandGroup(new ShooterControl(true), new Time(100), new BeltControl(true, false)),
        new Time(1000),
        new ParallelCommandGroup(new ShooterControl(false), new BeltControl(false, false))));
    Constants.AutoConstants.red3GPABEventMap.put("Stop", new SequentialCommandGroup(
        new GetOnRamp(),
        new AutoBalance()));
  }

  public void setRed1GPEventMap() {
    Constants.AutoConstants.red1GPEventMap.put("Start", new SequentialCommandGroup(
        new SequentialCommandGroup(new ShooterControl(true), new Time(100), new BeltControl(true, false)),
        new Time(800),
        new ParallelCommandGroup(new ShooterControl(false), new BeltControl(false, false)),
        new CustomSolControl(true),
        new ParallelCommandGroup(new BeltControl(true, false), new IntakeMotorControl(true, false))
        ));
    Constants.AutoConstants.red1GPEventMap.put("deployIntake", new SequentialCommandGroup(
        new Time(600),
        new ParallelCommandGroup(new BeltControl(false, false), new IntakeMotorControl(false, false)),
        new CustomSolControl(false)));
    Constants.AutoConstants.red1GPEventMap.put("Stop", new SequentialCommandGroup(
        new ShooterControl(1),
        new ParallelCommandGroup(new ShooterControl(true), new BeltControl(true, false)),
        new Time(1000),
        new ParallelCommandGroup(new ShooterControl(false), new BeltControl(false, false))));
  }

  public void setRed1GPABEventMap() {
    Constants.AutoConstants.red1GPABEventMap.put("Start", new SequentialCommandGroup(
        new SequentialCommandGroup(new ShooterControl(true), new Time(100), new BeltControl(true, false)),
        new Time(800),
        new ParallelCommandGroup(new ShooterControl(false), new BeltControl(false, false)),
        new CustomSolControl(true),
        new ParallelCommandGroup(new BeltControl(true, false), new IntakeMotorControl(true, false))
        ));
    Constants.AutoConstants.red1GPABEventMap.put("deployIntake", new SequentialCommandGroup(
        new Time(400),
        new ParallelCommandGroup(new BeltControl(false, false), new IntakeMotorControl(false, false)),
        new CustomSolControl(false)));
    Constants.AutoConstants.red1GPABEventMap.put("score2nd", new SequentialCommandGroup(
        new ShooterControl(1),
        new ParallelCommandGroup(new ShooterControl(true), new BeltControl(true, false)),
        new Time(1000),
        new ParallelCommandGroup(new ShooterControl(false), new BeltControl(false, false))));

    Constants.AutoConstants.red1GPABEventMap.put("Stop", new SequentialCommandGroup(
        new GetOnRamp(),
        new AutoBalance()));
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
<<<<<<< Updated upstream
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
=======
  private void configureButtonBindings() {
    YB.onTrue(new ChangeSpeed(0.8));
    XB.onTrue(new ChangeSpeed(0.6));
    AB.onTrue(new ChangeSpeed(0.45));

    BB.onTrue(new ToggleDirection());

    B5.onTrue(new ParallelCommandGroup(new ShooterControl(true), new BeltControl(true, false)));
    B5.onFalse(new ParallelCommandGroup(new ShooterControl(false), new BeltControl(false, false)));

    LBB.onTrue(new ParallelCommandGroup(new BeltControl(true, true), new IntakeMotorControl(true, true), new SolControl(true)));
    LBB.onFalse(new ParallelCommandGroup(new BeltControl(false, true), new IntakeMotorControl(false, true), new SolControl(false)));

    B4.onTrue(new ShooterControl(0));
    B4.onTrue(new BeltControl(0));
    B3.onTrue(new ShooterControl(1));
    B2.onTrue(new ShooterControl(2));
  
    rightButton.onTrue(new AutoAlign());

    // downbutton.onTrue(new SolControl(false));
    B7.onTrue(new ChangeLED());
>>>>>>> Stashed changes
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
<<<<<<< Updated upstream
<<<<<<< Updated upstream
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
=======
    // return m_chooser.getSelected();
    return new InstantCommand();
>>>>>>> Stashed changes
=======
    return m_chooser.getSelected();
>>>>>>> Stashed changes
  }
}
