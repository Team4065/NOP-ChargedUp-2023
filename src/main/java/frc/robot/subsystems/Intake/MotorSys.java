// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants;
<<<<<<< Updated upstream
=======
import frc.robot.RobotContainer;
import frc.robot.subsystems.Belt;
>>>>>>> Stashed changes

public class MotorSys extends SubsystemBase {
  CANSparkMax intakeMotor = new CANSparkMax(Constants.Intake.IntakeMotor, MotorType.kBrushless);
  boolean move;
  /** Creates a new MotorSys. */
  public MotorSys() {
    intakeMotor.setInverted(true);
  }

  public void set(boolean move) {
    this.move = move;
  }

  @Override
  public void periodic() {
<<<<<<< Updated upstream
    if (move == true) {
      intakeMotor.set(0.2);
    } else {
      intakeMotor.set(0);
=======
    RobotContainer.downButton = new POVButton(RobotContainer.XboxC, 180);

    if (RobotContainer.downButton.getAsBoolean() == false) {
      if (isIntake == false) {
        if (move == true) {
          intakeMotor.set(0.2);
        } else {
            intakeMotor.set(0);
        }
      } else {
        if (Belt.detectedColor.blue > Constants.Other.detectThreshold) {
          intakeMotor.set(0);
        } else {
          if (move == true) {
            intakeMotor.set(0.2);
          } else {
              intakeMotor.set(0);
          }
        }
      }
    } else if (RobotContainer.downButton.getAsBoolean() == true) {
      intakeMotor.set(-0.2);
>>>>>>> Stashed changes
    }
  }
}
