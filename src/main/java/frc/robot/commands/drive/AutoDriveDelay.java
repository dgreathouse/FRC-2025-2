package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.lib.g;


public class AutoDriveDelay extends Command {
  Timer m_Timer;
  double m_timeOut_sec;
  public AutoDriveDelay() {
    addRequirements(g.ROBOT.drive);
    m_timeOut_sec = 0;
    m_Timer = new Timer();
  }
  public AutoDriveDelay(double _timeOut_sec) {
    addRequirements(g.ROBOT.drive);
    m_timeOut_sec = _timeOut_sec;
    m_Timer = new Timer();
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Timer.restart();
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    g.ROBOT.drive.driveFieldCentric(0, 0, 0, g.ROBOT.angleActual_deg, g.DRIVETRAIN.ZERO_CENTER_OF_ROTATION_m);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_Timer.hasElapsed(m_timeOut_sec);
  }
}
