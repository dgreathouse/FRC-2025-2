package frc.robot.subsystems;

import static edu.wpi.first.units.Units.MetersPerSecond;
import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.lib.AI;
import frc.robot.lib.AprilTagAlignState;

import frc.robot.lib.CoralLiftState;
import frc.robot.lib.DriveMode;
import frc.robot.lib.IUpdateDashboard;
import frc.robot.lib.RobotAlignStates;
import frc.robot.lib.g;

public class Drivetrain extends SubsystemBase implements IUpdateDashboard {
  private SwerveDriveKinematics m_kinematics;
  private volatile SwerveDrivePoseEstimator m_poseEstimator;
  private PoseEstimatorThread m_poseEstimatorThread;
  private StatusSignal<Angle> m_yawStatusPigeon2;
  private StatusSignal<AngularVelocity> m_angularVelocityZStatusPigeon2;
  private double m_yawPrimary;
  private double m_angularVelocityZPrimary;
  private double m_yawSecondary = 0;
  private double m_angularVelocityZSecondary = 0;
  boolean m_isVisionEnabled = true;
  private double m_speedScale = 1.0;
  // TODO: Tune KP,KI,KD max output should be +/-1 Start around 1/3.14 for Kp
  private PIDController m_turnPID = new PIDController(0.45, 0.4, 0);

  private ChassisSpeeds m_speeds = new ChassisSpeeds();

  /** Creates a new Drivetrain. */
  @SuppressWarnings("unused")
  public Drivetrain() {
    m_yawStatusPigeon2 = g.ROBOT.gyro_pigeon2.getYaw();
    m_yawStatusPigeon2.setUpdateFrequency(g.SWERVE.CAN_UPDATE_FREQ_hz);
    m_angularVelocityZStatusPigeon2 = g.ROBOT.gyro_pigeon2.getAngularVelocityZDevice();
    m_angularVelocityZStatusPigeon2.setUpdateFrequency(g.SWERVE.CAN_UPDATE_FREQ_hz);
    g.ROBOT.gyro_navx.reset();
    g.SWERVE.modules[0] = new SwerveModule(
        "BR",
        12,
        true,
        22,
        true,
        2,
        -0.462,
        g.CHASSIS.BACK_RIGHT_SWERVE_X_POSITION_m,
        g.CHASSIS.BACK_RIGHT_SWERVE_Y_POSITION_m);
    g.SWERVE.modules[1] = new SwerveModule(
        "BL",
        13,
        false,
        23,
        true,
        3,
        -0.495,
        g.CHASSIS.BACK_LEFT_SWERVE_X_POSITION_m,
        g.CHASSIS.BACK_LEFT_SWERVE_Y_POSITION_m);
    g.SWERVE.modules[2] = new SwerveModule(
        "F",
        11,
        true,
        21,
        true,
        1,
        0.21753, // Gear on left of robot or right when looking at the front of the robot
        g.CHASSIS.FRONT_SWERVE_X_POSITION_m,
        g.CHASSIS.FRONT_SWERVE_Y_POSITION_m);
    if (g.SWERVE.COUNT == 4) {
      g.SWERVE.modules[3] = new SwerveModule("BL", 13, false, 23, false, 4, 0, 0, 0);
    }

    for (int i = 0; i < g.SWERVE.COUNT; i++) {
      g.SWERVE.positions[i] = new SwerveModulePosition();
      Robot.addDashboardUpdater(g.SWERVE.modules[i]);
    }
    updatePositions();

    m_kinematics = new SwerveDriveKinematics(
        g.SWERVE.modules[0].m_location,
        g.SWERVE.modules[1].m_location,
        g.SWERVE.modules[2].m_location);

    // m_odometry = new SwerveDriveOdometry(m_kinematics, g.ROBOT.angleActual_Rot2d,
    // g.SWERVE.positions);

    m_turnPID.enableContinuousInput(-Math.PI, Math.PI);
    // TODO set Derivative tolerance so atSetPoint only returns true at low speeds
    m_turnPID.setTolerance(Math.toRadians(1), Double.POSITIVE_INFINITY);
    m_turnPID.setIZone(Math.toRadians(20));
    m_turnPID.setIntegratorRange(-0.25, 0.25);

    m_poseEstimatorThread = new PoseEstimatorThread();
    m_poseEstimatorThread.start();

    m_poseEstimator = new SwerveDrivePoseEstimator(m_kinematics,
        g.ROBOT.angleActual_Rot2d,
        g.SWERVE.positions, new Pose2d(),
        g.DRIVETRAIN.STD_DEV_HIGH,
        g.DRIVETRAIN.STD_DEV_LOW);

    SmartDashboard.putBoolean("Robot/IsGyroPrimaryActive", true);
    
    SmartDashboard.putNumber("Drive/SpeedScale", m_speedScale);
    SmartDashboard.putData("Drive/AngleFCTurnPid",m_turnPID);
    Robot.addDashboardUpdater(this);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * Drive in old fashion mode. Forward on thumb stick makes the robot go forward
   * with reference to the front of the robot.
   * 
   * @param _xSpeed             The X speed in +/- 1.0 Forward +
   * @param _ySpeed             The Y speed in +/- 1.0 Left +
   * @param _rotate             The rotational speed in +/- 1.0
   * @param _centerOfRotation_m The center of rotation for the robot. To be the
   *                            ceter of the robot use
   *                            g.DRIVETRAIN.ZERO_CENTER_OF_ROTATION_m.
   */
  public void driveRobotCentric(double _xSpeed, double _ySpeed, double _rotate, Translation2d _centerOfRotation_m) {
    m_speeds.vxMetersPerSecond = _xSpeed * g.SWERVE.DRIVE.MAX_VELOCITY_mPsec * m_speedScale;
    m_speeds.vyMetersPerSecond = _ySpeed * g.SWERVE.DRIVE.MAX_VELOCITY_mPsec * m_speedScale;
    m_speeds.omegaRadiansPerSecond = _rotate * g.SWERVE.DRIVE.MAX_ANGULAR_VELOCITY_radPsec * m_speedScale;

    setSwerveModuleStates(m_speeds, _centerOfRotation_m);
  }

  /**
   * Drive in field centric mode. Forward on thumb stick is always forward on
   * field no matter what way the robot is facing
   * 
   * @param _xSpeed             The X speed in +/- 1.0 Forward +
   * @param _ySpeed             The Y speed in +/- 1.0 Left +
   * @param _rotate             The rotational speed in +/- 1.0
   * @param _robotAngle_deg     The current robot angle
   * @param _centerOfRotation_m The center of rotation for the robot. To be the
   *                            ceter of the robot use
   *                            g.DRIVETRAIN.ZERO_CENTER_OF_ROTATION_m.
   */
  public void driveFieldCentric(double _xSpeed, double _ySpeed, double _rotate, double _robotAngle_deg,
      Translation2d _centerOfRotation_m) {

    m_speeds.vxMetersPerSecond = _xSpeed * g.SWERVE.DRIVE.MAX_VELOCITY_mPsec * m_speedScale;
    m_speeds.vyMetersPerSecond = _ySpeed * g.SWERVE.DRIVE.MAX_VELOCITY_mPsec * m_speedScale;
    m_speeds.omegaRadiansPerSecond = _rotate * g.SWERVE.DRIVE.MAX_ANGULAR_VELOCITY_radPsec * m_speedScale;
    m_speeds = ChassisSpeeds.fromRobotRelativeSpeeds(m_speeds, new Rotation2d(Math.toRadians(-_robotAngle_deg)));
     setSwerveModuleStates(m_speeds, _centerOfRotation_m);
  }

  /**
   * Use the X and Y values that usually come from a joystick and drive with a
   * robot target angle.
   * This maintains the robot facing direction to the target angle.
   * 
   * @param _xSpeed             The X speed in +/- 1.0
   * @param _ySpeed             The Y speed in +/- 1.0
   * @param _robotAngle_deg     The current robot angle
   * @param _targetAngle_deg    The desired angle for the front of the robot to
   *                            face.
   * @param _centerOfRotation_m The center of rotation for the robot. To be the
   *                            ceter of the robot use
   *                            g.DRIVETRAIN.ZERO_CENTER_OF_ROTATION_m.
   */
  public void driveAngleFieldCentric(double _xSpeed, double _ySpeed, double _robotAngle_deg, double _targetAngle_deg, Translation2d _centerOfRotation_m) {
    m_speeds.vxMetersPerSecond = _xSpeed * g.SWERVE.DRIVE.MAX_VELOCITY_mPsec * m_speedScale;
    m_speeds.vyMetersPerSecond = _ySpeed * g.SWERVE.DRIVE.MAX_VELOCITY_mPsec * m_speedScale;

    double rotate = m_turnPID.calculate(Math.toRadians(_robotAngle_deg), Math.toRadians(_targetAngle_deg));
    SmartDashboard.putNumber("Rotate", rotate);
    // rotate = MathUtil.applyDeadband(rotate, g.DRIVETRAIN.TURN_DEADBAND_rad);
    m_speeds.omegaRadiansPerSecond = rotate * g.SWERVE.DRIVE.MAX_ANGULAR_VELOCITY_radPsec * m_speedScale;
    m_speeds = ChassisSpeeds.fromRobotRelativeSpeeds(m_speeds, new Rotation2d(Math.toRadians(-_robotAngle_deg)));

    setSwerveModuleStates(m_speeds, _centerOfRotation_m);
  }

  /**
   * Get the X and Y values from the Drive Angle and call
   * {@link #driveAngleFieldCentric(double, double, double, double)}
   * 
   * 
   * @param _maxSpeed           The max speed to drive at with range 0.0 to 1.0
   * @param _robotAngle_deg     The angle of the robot which usually is
   *                            g.ROBOT.angleActual_deg
   * @param _targetAngle_deg    The angle you want the robot front to point to.
   * @param _driveAngle_deg     The drive angle you want the robot to drive at
   * @param _centerOfRotation_m The center of rotation for the robot. To be the
   *                            ceter of the robot use
   *                            g.DRIVETRAIN.ZERO_CENTER_OF_ROTATION_m.
   */
  public void drivePolarFieldCentric(double _maxSpeed, double _robotAngle_deg, double _targetAngle_deg,
      double _driveAngle_deg, Translation2d _centerOfRotation_m) {

    double y = Math.sin(Math.toRadians(_driveAngle_deg)) * _maxSpeed;
    double x = Math.cos(Math.toRadians(_driveAngle_deg)) * _maxSpeed;
    driveAngleFieldCentric(x, y, _robotAngle_deg, _targetAngle_deg, _centerOfRotation_m);
  }
  // /** This is a compensated AngleFieldCentric that takes the speed from the x,y
  // and generates a new X,Y from driveAngle
  // *
  // * @param _xSpeed The X speed in +/- 1.0
  // * @param _ySpeed The Y speed in +/- 1.0
  // * @param _robotAngle_deg The current robot angle
  // * @param _targetAngle_deg The desired angle for the front of the robot to
  // face.
  // * @param _driveAngle_deg The angle to drive the robot at on the field
  // * @param _centerOfRotation_m The center of rotation for the robot. To be the
  // ceter of the robot use g.DRIVETRAIN.ZERO_CENTER_OF_ROTATION_m.
  // */
  // public void driveAngleFieldCentric(double _xSpeed, double _ySpeed, double
  // _robotAngle_deg, double _targetAngle_deg, double _driveAngle_deg,
  // Translation2d _centerOfRotation_m){

  // double speed = Math.max(Math.abs(_xSpeed), Math.abs(_ySpeed));
  // double y = Math.sin(Math.toRadians(_driveAngle_deg)) * speed;
  // double x = Math.cos(Math.toRadians(_driveAngle_deg)) * speed;
  // driveAngleFieldCentric(x, y, _robotAngle_deg, _targetAngle_deg,
  // _centerOfRotation_m);
  // }

  public void setSwerveModuleStates(ChassisSpeeds _speeds, Translation2d _centerOfRotation_m) {
    _centerOfRotation_m = _centerOfRotation_m == null ? g.DRIVETRAIN.ZERO_CENTER_OF_ROTATION_m : _centerOfRotation_m;

    SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(_speeds, _centerOfRotation_m);

    //SwerveDriveKinematics.desaturateWheelSpeeds(states, MetersPerSecond.of(g.SWERVE.DRIVE.MAX_VELOCITY_mPsec));
    g.DRIVETRAIN.driveSpeedRequested_mps = 0.0;
    for (int i = 0; i < g.SWERVE.COUNT; i++) {
      g.SWERVE.modules[i].setDesiredState(states[i]);
      g.DRIVETRAIN.driveSpeedRequested_mps += Math.abs(states[i].speedMetersPerSecond);
    }
    g.DRIVETRAIN.driveSpeedRequested_mps = g.DRIVETRAIN.driveSpeedRequested_mps / g.SWERVE.COUNT;
    g.DRIVETRAIN.driveSpeedActual_mps = getDriveSpeed();
    g.DRIVETRAIN.driveSpeedError_mps = Math.abs(g.DRIVETRAIN.driveSpeedActual_mps - g.DRIVETRAIN.driveSpeedActual_mps);
  }

  /**
   * Set the drive angle for AngleFieldCentric mode if the
   * Hypotenus of the x,y is greater that a threashold
   * 
   * @param _x The X value which is Forward Positive
   * @param _y The Y value which is Left Positive
   */
  public void setTargetRobotAngle(double _x, double _y) {

    double x = _x;
    double y = _y;
    double hyp = Math.hypot(x, y); // Always positive
    double joystickAngle = Math.toDegrees(Math.atan2(y, x));

    if (Math.abs(hyp) > g.OI.THUMBSTICK_AXIS_ANGLE_DEADBAND) {
      if (joystickAngle >= -22.5 && joystickAngle <= 22.5) { // North
        setTargetRobotAngle(RobotAlignStates.FRONT);
      } else if (joystickAngle >= -67.5 && joystickAngle < -22.5) { // North East
        setTargetRobotAngle(RobotAlignStates.FRONT_LEFT);
      } else if (joystickAngle >= -112.5 && joystickAngle < -67.5) { // East
        setTargetRobotAngle(RobotAlignStates.RIGHT);
      } else if (joystickAngle >= -157.5 && joystickAngle < -112.5) { // South East
        setTargetRobotAngle(RobotAlignStates.BACK_LEFT);
      } else if ((joystickAngle >= 157.5 && joystickAngle <= 180.0)
          || (joystickAngle <= -157.5 && joystickAngle > -179.99)) { // South
        setTargetRobotAngle(RobotAlignStates.BACK);
      } else if (joystickAngle <= 67.5 && joystickAngle > 22.5) { // North West
        setTargetRobotAngle(RobotAlignStates.FRONT_RIGHT);
      } else if (joystickAngle <= 112.5 && joystickAngle > 67.5) { // West
        setTargetRobotAngle(RobotAlignStates.LEFT);
      } else if (joystickAngle <= 157.5 && joystickAngle > 112.5) { // South West
        setTargetRobotAngle(RobotAlignStates.BACK_RIGHT);
      }
    }
  }
  public void setAITargetRobotAngle(double _x, double _y) {
    
    double x = _x;
    double y = _y;
    double hyp = Math.hypot(x, y); // Always positive
    double joystickAngle = Math.toDegrees(Math.atan2(y, x));

    if (Math.abs(hyp) > g.OI.THUMBSTICK_AXIS_ANGLE_DEADBAND) {
      if (joystickAngle >= -30 && joystickAngle <= 30) { // North
        setTargetRobotAngle(RobotAlignStates.FRONT);
      } else if (joystickAngle >= -90 && joystickAngle < -30) { // North East
        setTargetRobotAngle(RobotAlignStates.FRONT_LEFT);
      } else if (joystickAngle >= -150 && joystickAngle < -90) { // South East
        setTargetRobotAngle(RobotAlignStates.BACK_LEFT);
      } else if ((joystickAngle >= 150 && joystickAngle <= 180.0)
          || (joystickAngle <= -150 && joystickAngle > -180)) { // South
        setTargetRobotAngle(RobotAlignStates.BACK);
      } else if (joystickAngle <= 90 && joystickAngle > 30) { // North West
        setTargetRobotAngle(RobotAlignStates.FRONT_RIGHT);
      } else if (joystickAngle <= 150 && joystickAngle > 90) { // South West
        setTargetRobotAngle(RobotAlignStates.BACK_RIGHT);
      }
    }
  }
  public void setAITargetRobotAngle(RobotAlignStates _state){
    switch (_state) {
      case BACK:
        AI.StateInput.setState(RobotAlignStates.BACK);
        break;
      case BACK_LEFT:
        AI.StateInput.setState(RobotAlignStates.BACK_LEFT);
        break;
      case BACK_RIGHT:
        AI.StateInput.setState(RobotAlignStates.BACK_RIGHT);
        break;
      case FRONT:
        AI.StateInput.setState(RobotAlignStates.FRONT);
        break;
      case FRONT_LEFT:
        AI.StateInput.setState(RobotAlignStates.FRONT_LEFT);
        break;
      case FRONT_RIGHT:
        AI.StateInput.setState(RobotAlignStates.FRONT_RIGHT);
        break;
      default:
        g.ROBOT.angleRobotTarget_deg = setTargetRobotAngle(0.0);
        break;
    }
  }
  public void setDefenseTargetRobotAngle(double _x, double _y){
    double x = _x;
    double y = _y;
    double hyp = Math.hypot(x, y); // Always positive
    double joystickAngle = Math.toDegrees(Math.atan2(y, x));

    if (Math.abs(hyp) > g.OI.THUMBSTICK_AXIS_ANGLE_DEADBAND) {
      if (joystickAngle >= 0) { 
        setTargetRobotAngle(36);
      } else if (joystickAngle < 0 ) { // North East
        setTargetRobotAngle(-36);
      }
    }
  }
  /*
   * There are the following states for the robot to react to.
   * 1. g.ROBOT.alignmentState
   * a. Set by buttons in teleop
   * b. Set by autonomous commands
   * 2. g.CORAL.armState
   * 3. g.VISION.aprilTagAlignState
   * 
   * Order that the states are changed are the problem.
   * 
   * One wat to solve this is to have one method that is called with all the
   * states.
   */
  /**
   * Set the target angle for the robot to drive to in AngleFieldCentric mode
   * 
   * @param _state
   * @return
   */
  public double setTargetRobotAngle(RobotAlignStates _state) {
     g.DRIVETRAIN.driveMode = DriveMode.ANGLE_FIELD_CENTRIC;
     g.ROBOT.alignmentState = _state;
    switch (_state) {
      case BACK:
        g.ROBOT.angleRobotTarget_deg = setTargetRobotAngle(180.0);
        break;
      case BACK_LEFT:
        g.ROBOT.angleRobotTarget_deg = setTargetRobotAngle(-120.0);
        break;
      case BACK_RIGHT:
        g.ROBOT.angleRobotTarget_deg = setTargetRobotAngle(120.0);
        break;
      case FRONT:
        g.ROBOT.angleRobotTarget_deg = setTargetRobotAngle(0.0);
        break;
      case FRONT_LEFT:
        g.ROBOT.angleRobotTarget_deg = setTargetRobotAngle(-60.0);
        break;
      case FRONT_RIGHT:
        g.ROBOT.angleRobotTarget_deg = setTargetRobotAngle(60.0);
        break;
      case LEFT:
        g.ROBOT.angleRobotTarget_deg = setTargetRobotAngle(90.0);
        break;
      case RIGHT:
        g.ROBOT.angleRobotTarget_deg = setTargetRobotAngle(-90.0);
        break;
      case STATION_LEFT:
        g.CORALLIFT.state = CoralLiftState.START;
        g.ROBOT.angleRobotTarget_deg = setTargetRobotAngle(-54.0);
        break;
      case STATION_RIGHT:
        g.CORALLIFT.state = CoralLiftState.START;
        g.ROBOT.angleRobotTarget_deg = setTargetRobotAngle(54.0);
        break;
        case STATION_LEFT_DEFENSE:
        g.CORALLIFT.state = CoralLiftState.START;
        g.ROBOT.angleRobotTarget_deg = setTargetRobotAngle(-36.0);
        break;
      case STATION_RIGHT_DEFENSE:
        g.CORALLIFT.state = CoralLiftState.START;
        g.ROBOT.angleRobotTarget_deg = setTargetRobotAngle(36.0);
        break;
      case UNKNOWN:
        g.ROBOT.alignmentState = RobotAlignStates.UNKNOWN;
        g.ROBOT.angleRobotTarget_deg = setTargetRobotAngle(0.0);
        break;
      default:
        g.ROBOT.alignmentState = RobotAlignStates.UNKNOWN;
        g.ROBOT.angleRobotTarget_deg = setTargetRobotAngle(0.0);
        break;

    }
    return g.ROBOT.angleRobotTarget_deg;
  }

  /**
   * Directly set the g.ROBOT.angleTarget_deg to a angle. Generally this is used
   * in autonomous or by a button.
   *
   * @param _angle_deg The target angle the robot should PID to in
   *                   AngleFieldCentric Mode.
   */
  public double setTargetRobotAngle(double _angle_deg) {
    if (DriverStation.getAlliance().isPresent()) {
      if (DriverStation.getAlliance().get() == Alliance.Red) {
        _angle_deg = _angle_deg - 180;
      }
    }
    g.ROBOT.angleRobotTarget_deg = _angle_deg;
    return g.ROBOT.angleRobotTarget_deg;
  }

  /**
   * Is the robot rotational on target for g.ROBOT.AngleTarget_deg
   *
   * @return if the rotation of the robot is on target
   */
  public boolean isRotateAtTarget() {
    g.DRIVETRAIN.turnPIDErrorDerivative = m_turnPID.getErrorDerivative();
    return m_turnPID.atSetpoint();
  }

  public void resetYaw(double _angle) {
    // if(DriverStation.getAlliance().isPresent()){
    // if(DriverStation.getAlliance().get() == Alliance.Red){
    // _angle = _angle - 180;
    // }
    // }
    g.ROBOT.gyro_pigeon2.setYaw(_angle);
    
    g.ROBOT.gyro_navx.setAngleAdjustment(-_angle);
  }

  private void updatePositions() {
    for (int i = 0; i < g.SWERVE.COUNT; i++) {
      g.SWERVE.positions[i] = g.SWERVE.modules[i].updatePosition();
    }
  }

  public double getYaw() {
    return g.ROBOT.isPrimaryGyroActive ? m_yawPrimary : m_yawSecondary;
  }

  public double getAngularVelocityZ() {
    return g.ROBOT.isPrimaryGyroActive ? m_angularVelocityZPrimary : m_angularVelocityZSecondary;
  }

  // public void setOdometry(StartLocation _start) {
  //   switch (_start) {
  //     case LEFT:
  //       m_poseEstimator.resetPose(g.ROBOT.POSE_START_LEFT);
  //       break;
  //     case RIGHT:
  //       m_poseEstimator.resetPose(g.ROBOT.POSE_START_RIGHT);
  //       break;
  //     case CENTER:
  //       m_poseEstimator.resetPose(g.ROBOT.POSE_START_CENTER);
  //       break;
  //     case ZERO:
  //       m_poseEstimator.resetPose(g.ROBOT.POSE_START_ZERO);
  //       break;
  //     default:
  //       m_poseEstimator.resetPose(g.ROBOT.POSE_START_LEFT);
  //       break;
  //   }
  // }

  public void toggleSpeed() {
    if (g.ROBOT.drive.getSpeedScale() < 0.65) {
      g.ROBOT.drive.setSpeedScale(0.75);

    } else {
      g.ROBOT.drive.setSpeedScale(0.50);
    }
  }

  public String getSpeedMode() {
    if (g.ROBOT.drive.getSpeedScale() < 0.65) {
      return "LOW";
    } else {
      return "HIGH";
    }
  }

  public void resetOdometry(Pose2d _pose) {
    m_poseEstimator.resetPose(_pose);
  }

  public void setAprilTagAlignment(AprilTagAlignState _alignState) {
    g.VISION.aprilTagAlignState = _alignState;
    setTargetRobotAngle(g.ROBOT.alignmentState);
    if(g.ROBOT.alignmentState == RobotAlignStates.STATION_LEFT || g.ROBOT.alignmentState == RobotAlignStates.STATION_RIGHT){
      AI.ReefModel.resetReefState(_alignState);
    }
  }

  public double getDriveSpeed() {
    double speed = 0.0;
    
    for (int i = 0; i < g.SWERVE.COUNT; i++) {
      speed = speed + g.SWERVE.modules[i].getDriveSpeed();
    }
    speed = speed / g.SWERVE.COUNT;
    return speed;
  }

  public double getDriveSpeedError(double _speed) {
    return Math.abs(_speed - g.DRIVETRAIN.driveSpeedActual_mps);
  }

  public double getSpeedScale() {
    return m_speedScale;
  }

  public void setSpeedScale(double _scale) {
    m_speedScale = _scale;
  }

  private class PoseEstimatorThread extends Thread {
    public PoseEstimatorThread() {
      super();
    }

    @Override
    public void run() {
      while (true) {
        updatePositions();

        m_yawStatusPigeon2 = g.ROBOT.gyro_pigeon2.getYaw();
        m_angularVelocityZStatusPigeon2 = g.ROBOT.gyro_pigeon2.getAngularVelocityZDevice();
        m_yawSecondary = m_yawStatusPigeon2.getValueAsDouble();
        m_angularVelocityZSecondary = m_angularVelocityZStatusPigeon2.getValueAsDouble();

        m_yawPrimary = -g.ROBOT.gyro_navx.getAngle();
        m_angularVelocityZPrimary = -g.ROBOT.gyro_navx.getVelocityZ();

        g.ROBOT.angleActual_deg = getYaw();
        g.ROBOT.angleActual_Rot2d = Rotation2d.fromDegrees(g.ROBOT.angleActual_deg);

        g.ROBOT.vision.calculatePose();

        g.ROBOT.pose2d = m_poseEstimator.update(g.ROBOT.angleActual_Rot2d, g.SWERVE.positions);
        g.ROBOT.field2d.setRobotPose(g.ROBOT.pose2d);

        g.ROBOT.pose3d = new Pose3d(g.ROBOT.pose2d);

        try {
          Thread.sleep(g.ROBOT.ODOMETRY_RATE_ms);
        } catch (InterruptedException e) {
          System.out.println(e.getMessage());
        }

      }
    }
  }

  /**
   * Add a vision measurement to the PoseEstimator.
   * This only gets called if a vision measurement is available and with low
   * ambiguity.
   * 
   * @param _estPose   The estimated pose from the vision system
   * @param _timeStamp The time the vision measurement was taken
   */
  public void addVisionMeasurement(Pose2d _estPose, double _timeStamp) {
    // TODO only add vision measurement if drive speed is low. TEST
    // [x] Add speed check
    // [ ] Test if this works

    if (g.DRIVETRAIN.driveSpeedActual_mps < g.DRIVETRAIN.DRIVE_SPEED_LOW_mps) { // If the robot is moving slow
      if (m_isVisionEnabled) {
        // m_poseEstimator.setVisionMeasurementStdDevs(g.DRIVETRAIN.STD_DEV_HIGH); //
        // Set the standard deviation to high for Vision
        // if(g.ROBOT.vision.isYawResetComplete())
        if(g.ROBOT.vision.getTargetDistance() > g.VISION.TARGET_DISTANCE_MIN_m && g.ROBOT.vision.getTargetDistance() < g.VISION.TARGET_DISTANCE_MAX_m){ // If the vision target is far enough away
          m_poseEstimator.addVisionMeasurement(_estPose, _timeStamp); // Add the vision measurement to the PoseEstimator
        }
       // m_poseEstimator.addVisionMeasurement(_estPose, _timeStamp); // Add the vision measurement to the PoseEstimator
      }
    } else {
      // m_poseEstimator.setVisionMeasurementStdDevs(g.DRIVETRAIN.STD_DEV_LOW); // Set
      // the standard deviation to high for Vision
    }

  }

  public boolean getIsVisionEnabled() {
    return m_isVisionEnabled;
  }

  public void setIsVisionEnabled(boolean _enabled) {
    m_isVisionEnabled = _enabled;
  }

  public void updateDashboard() {
    // g.SWERVE.totalSwerveCurrent_amps = 0;
    // for (SwerveModule swerveModule : g.SWERVE.modules) {
    // g.SWERVE.totalSwerveCurrent_amps += Math.abs(swerveModule.getDriveCurrent())
    // + Math.abs(swerveModule.getSteerCurrent());
    // }

    SmartDashboard.putData("Robot/Drive Field2d", g.ROBOT.field2d);
    SmartDashboard.putBoolean("Robot/IsVisionEnabled", m_isVisionEnabled);
    SmartDashboard.putNumber("Robot/Pose Angle", g.ROBOT.pose2d.getRotation().getDegrees());
    SmartDashboard.putNumber("Robot/angleTarget_deg", g.ROBOT.angleRobotTarget_deg);
    SmartDashboard.putNumber("Robot/angleActual_deg", g.ROBOT.angleActual_deg);
    SmartDashboard.putNumber("Robot/GyroPrimary_deg", m_yawPrimary);
    SmartDashboard.putNumber("Robot/GyroSecondary_deg", m_yawSecondary);
    SmartDashboard.putBoolean("Robot/Is AprilTag Active", g.ROBOT.vision.getIsAutoAprilTagActive());
    SmartDashboard.putString("Robot/AlignState", g.ROBOT.alignmentState.toString());
    SmartDashboard.putNumber("Robot/PD_Volts", g.ROBOT.pd.getVoltage());
    SmartDashboard.putNumber("Robot/PD_Amps", g.ROBOT.pd.getTotalCurrent());

    SmartDashboard.putNumber("Drive/Speed Actual", g.DRIVETRAIN.driveSpeedActual_mps);
    SmartDashboard.putNumber("Drive/Speed Requested", g.DRIVETRAIN.driveSpeedRequested_mps);
    SmartDashboard.putNumber("Drive/Speed Error", getDriveSpeedError(getDriveSpeed()));
    SmartDashboard.putString("Drive/Speed Mode", getSpeedMode());
    SmartDashboard.putString("Drive/Drive Mode", g.DRIVETRAIN.driveMode.toString());
    SmartDashboard.putNumber("Drive/TurnPID error Derivative", g.DRIVETRAIN.turnPIDErrorDerivative);

    SmartDashboard.putData("Drive/Subsystem", this);
    // Get from Dashboard
    g.ROBOT.isPrimaryGyroActive = SmartDashboard.getBoolean("Robot/IsGyroPrimaryActive", true);


  }
}
