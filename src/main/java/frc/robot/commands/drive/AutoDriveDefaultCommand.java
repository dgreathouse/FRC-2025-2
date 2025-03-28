package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.lib.g;

public class AutoDriveDefaultCommand extends Command {
  /** Creates a new AutoDriveDefaultCommand. */
  public AutoDriveDefaultCommand() {
    addRequirements(g.ROBOT.drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   // g.SWERVE.isEnabled = false;
   g.ROBOT.drive.setServeModulesStatesOff();
    g.ROBOT.drive.driveFieldCentric(0, 0, 0, g.ROBOT.angleActual_deg, g.DRIVETRAIN.ZERO_CENTER_OF_ROTATION_m);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
