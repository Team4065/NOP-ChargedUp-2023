// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.subsystems.PhotonVision;

// public class tagInfo extends CommandBase {
//   /** Creates a new tagInfo. */
//   boolean FIN = false;
//   PhotonVision m_PhotonVision;
//   public tagInfo(PhotonVision m_PhotonVision) {
//     // Use addRequirements() here to declare subsystem dependencies.
//     this.m_PhotonVision = m_PhotonVision;
//     addRequirements(m_PhotonVision);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {}

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     System.out.println("target = " + m_PhotonVision.hasTargets());
//     System.out.println("area = "+ m_PhotonVision.getArea());
//     System.out.println("yaw = "+ m_PhotonVision.getYaw());
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {}

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return FIN;
//   }
// }
