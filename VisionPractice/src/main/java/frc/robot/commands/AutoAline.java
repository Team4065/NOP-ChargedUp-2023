// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.photonvision2;

public class AutoAline extends CommandBase {
  /** Creates a new AutoAline. */
  PIDController pid = new PIDController(0.01, 0, 0);
  double xTargetPos = photonvision2.tagYaw();
  double targetArea = photonvision2.tagArea();
  // double x = 1.2;
  boolean ready = false;
  boolean FIN;
  public AutoAline() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.d_drivetrain);
    addRequirements(RobotContainer.p_photonvision2);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    targetArea = photonvision2.tagArea();

    while (RobotContainer.p_photonvision2.tagArea() == 20) {
      RobotContainer.p_photonvision2.setReady();
    }

    RobotContainer.d_drivetrain.setRight(pid.calculate(targetArea, 20));
    RobotContainer.d_drivetrain.setleft(pid.calculate(targetArea, 20));
    System.out.println("Area = "+targetArea);
    

    if (RobotContainer.AB.getAsBoolean() == true) {

    } else {
      FIN = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return FIN;
  }
}
