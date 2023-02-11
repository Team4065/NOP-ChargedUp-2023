// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.photonvision2;

public class infoFromLibrary extends CommandBase {
  /** Creates a new infoFromLibrary. */
  boolean FIN = false;
  photonvision2 m_photonvision2;
  public infoFromLibrary(photonvision2 m_photonvision2) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_photonvision2 = m_photonvision2;
    addRequirements(m_photonvision2);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_photonvision2.hastargetstest() == true) {
      System.out.println("has targets = "+m_photonvision2.hastargetstest());
      System.out.println("target ID = "+m_photonvision2.targetID());
      System.out.println("area = "+m_photonvision2.tagArea());
      System.out.println("pitch = "+m_photonvision2.tagpitch());
      System.out.println("yaw = "+m_photonvision2.tagYaw());
    }
    else if (m_photonvision2.hastargetstest() == false) {
      System.out.println("no targets");
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
