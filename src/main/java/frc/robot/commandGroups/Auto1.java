// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.coralLift.CoralMoveToStateCommand;
import frc.robot.commands.coralLift.CoralSetStateCommand;
import frc.robot.commands.coralLift.CoralSpinOutCommand;
import frc.robot.commands.drive.AutoDriveDelay;
import frc.robot.commands.drive.AutoDriveToPose;
import frc.robot.commands.drive.AutoRotateToPose;
import frc.robot.lib.AprilTagAlignState;
import frc.robot.lib.CoralLiftState;
import frc.robot.lib.g;

public class Auto1 extends SequentialCommandGroup {
  int tagID;
  AprilTagAlignState aprilTagAlignState;
  CoralLiftState coralLiftState;

  public Auto1(int _tagID, AprilTagAlignState _aprilTagAlignState, CoralLiftState _coralLiftState) {
    tagID = _tagID;
    aprilTagAlignState = _aprilTagAlignState;
    coralLiftState = _coralLiftState;
    
    addCommands(
      new AutoDriveDelay(),
      new AutoRotateToPose(g.ROBOT.vision.getRobotPoseForAprilTag(tagID, aprilTagAlignState), .3, 1),
      new ParallelCommandGroup(
        new CoralMoveToStateCommand(_coralLiftState, 1.75),
        new AutoDriveToPose(g.ROBOT.vision.getRobotPoseForAprilTag(tagID, aprilTagAlignState), 0.5, 1.75)
        ),
      new CoralSpinOutCommand(coralLiftState, 0.75),
      new CoralSetStateCommand(CoralLiftState.START) 
    );
  }
}
