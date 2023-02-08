// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class BeamBreaks extends SubsystemBase {
  /** Creates a new BeamBreaks. */
  public BeamBreaks() {}

  public DigitalInput BeamBk = new DigitalInput(0);

  boolean beamBreak = false;


  public boolean getBeamBreak(int BeamBreakNumber) {
    beamBreak = BeamBk.get();
    return beamBreak;
  } 

  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
