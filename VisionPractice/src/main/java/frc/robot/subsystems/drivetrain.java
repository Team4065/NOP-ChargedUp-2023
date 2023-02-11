// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


public class drivetrain extends SubsystemBase {
  /** Creates a new drivetrain. */
  public drivetrain() {
    m_right.setInverted(true);
  }

  CANSparkMax m_right = new CANSparkMax(1, MotorType.kBrushless);
  CANSparkMax m_left = new CANSparkMax(2, MotorType.kBrushless);

  public void setleft(double speed) {
    m_left.set(speed);
  }

  public void setRight(double speed) {
    m_right.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
