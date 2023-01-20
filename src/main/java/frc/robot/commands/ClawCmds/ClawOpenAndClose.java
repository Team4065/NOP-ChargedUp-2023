// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ClawCmds;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Time;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ClawOpenAndClose extends SequentialCommandGroup {
  /** Creates a new ClawOpenAndClose. */
  public ClawOpenAndClose(int delay, double openSpeed, double closeSpeed) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new ClawCmd(openSpeed), new Time(delay), new ClawCmd(0), new Time(1000), new ClawCmd(closeSpeed), new Time(delay), new ClawCmd(0));
    // addCommands(new ClawCmd(0.6), new Time(100));
  }
}