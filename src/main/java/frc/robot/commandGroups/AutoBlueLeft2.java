// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.commands.drive.AutoDriveToPose;
import frc.robot.commands.drive.AutoDriveDelay;
import frc.robot.lib.AprilTagAlignState;
import frc.robot.lib.CoralArmState;
import frc.robot.lib.g;


public class AutoBlueLeft2 extends SequentialCommandGroup {
  public double m_delay;
  /** Creates a new AutoBlueRight2. */
  public AutoBlueLeft2(double _delay) {
    m_delay = _delay;

    addCommands(
      // new AutoDriveDelay(m_delay),
      // new ParallelCommandGroup(
      //   new AutoDriveToPose(g.ROBOT.vision.getRobotPoseForAprilTag(20, AprilTagAlignState.RIGHT), 0.4, 5),
      //   new CoralMoveToStateCommand(CoralArmState.L2)
      // ),
      // new CoralSpinOutCommand(CoralArmState.L2, 1.5)

    );
  }
}
