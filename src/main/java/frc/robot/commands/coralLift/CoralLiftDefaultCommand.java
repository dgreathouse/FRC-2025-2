// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.coralLift;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.lib.g;

public class CoralLiftDefaultCommand extends Command {

  public CoralLiftDefaultCommand() {
    addRequirements(g.ROBOT.coralLift);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    // Intake Spinner speeds
    if(g.OI.DRIVER_CORAL_IN.getAsBoolean()){
      g.ROBOT.coralLift.spin(0.15);
    }else if(g.OI.DRIVER_CORAL_OUT.getAsBoolean()){
      g.ROBOT.coralLift.spinOut(g.CORALLIFT.state);
    }else {
      g.ROBOT.coralLift.spin(0);
    }
    // Intake Arm Angle
    g.ROBOT.coralLift.rotate(g.CORALLIFT.state);
    // Lift Height
    g.ROBOT.coralLift.moveLiftToPosition(g.CORALLIFT.state);

  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
