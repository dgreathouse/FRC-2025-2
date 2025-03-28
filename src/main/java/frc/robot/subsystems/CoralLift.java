// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.lib.CoralLiftState;
import frc.robot.lib.IUpdateDashboard;
import frc.robot.lib.g;

public class CoralLift extends SubsystemBase implements IUpdateDashboard {

  TalonFXS m_leftMotor = new TalonFXS(g.CAN_IDS_ROBORIO.CORAL_LEFT_MOTOR);
  TalonFXS m_rightMotor = new TalonFXS(g.CAN_IDS_ROBORIO.CORAL_RIGHT_MOTOR);
  SparkMax m_rotateMotor = new SparkMax(g.CAN_IDS_ROBORIO.CORAL_ROTATE_MOTOR, MotorType.kBrushless);
  TalonFX m_liftMotor = new TalonFX(g.CAN_IDS_ROBORIO.LIFT_MOTOR, g.CAN_IDS_ROBORIO.NAME);

  PIDController m_rotatePID;
  PIDController m_liftPID;

  VoltageOut m_leftVoltageOut = new VoltageOut(0.0).withEnableFOC(true);
  VoltageOut m_rightVoltageOut = new VoltageOut(0.0).withEnableFOC(true);
  VoltageOut m_liftVoltageOut = new VoltageOut(0).withEnableFOC(true);

  double m_maxLiftUpVolts = 9;
  double m_maxLiftDownVolts = -8;
  double m_rotateStartAngle_deg = -67.5;

  public CoralLift() {
    m_rotatePID = new PIDController(3.5, 0, 0);
    // m_rotatePID.setIZone(Math.toRadians(7.5));
    // m_rotatePID.setIntegratorRange(-.1, .1);
    // m_rotatePID.setTolerance(Math.toRadians(1));
    
    m_liftPID = new PIDController(0.15, 0, 0);
    m_liftPID.setIZone(20); // Sets the IZone range.
    m_liftPID.setIntegratorRange(-.1, .1); // Sets the Integrator range.
    m_liftPID.setTolerance(0.1); // Sets the tolerance
    SmartDashboard.putData("RotatePID", m_rotatePID);
    configureMotors();
    Robot.addDashboardUpdater(this);
  }

  public void spinOut(CoralLiftState _state) {
    double speed = 0;
    switch (_state) {
      case ALGAE_HIGH:
        speed = 0.5;
        break;
      case ALGAE_LOW:
        speed = 0.5;
        break;
      case L1:
        speed = 0.1;
        break;
      case L2:
        speed = 0.175;
        break;
      case L3:
        speed = 0.175;
        break;
      case START:
        break;
      default:
        break;
    }
    m_leftMotor.setControl(m_leftVoltageOut.withOutput(speed * g.ROBOT.MAX_BATTERY_SUPPLY_volts));
    m_rightMotor.setControl(m_rightVoltageOut.withOutput(-speed * g.ROBOT.MAX_BATTERY_SUPPLY_volts));
  }

  public void rotate(CoralLiftState _state) {
    switch (_state) { // TODO: adjust the angles for the levels
      case L1:
        rotateToAngle(55);
        break;
      case L2:
        rotateToAngle(40);
        break;
      case L3:
        rotateToAngle(30);
        break;
      case ALGAE_HIGH:
        rotateToAngle(55);
        break;
      case ALGAE_LOW:
        rotateToAngle(60);
        break;
      case START:
        rotateToAngle(m_rotateStartAngle_deg);
        break;
        case ZERO:
        rotateToAngle(0);
        break;
      case LIFT_CLIMB_UP:
        rotateToAngle(55);
        break;
      case LIFT_CLIMB_DOWN:
        rotateToAngle(55);
        break;
      default:
        break;
    }
  }
  double m_L3Position_mm = 416;
  public void moveLiftToPosition(CoralLiftState _state) {
    switch (_state) {
      case L1:
        moveToPosition(0);
        break;
      case L2:
        moveToPosition(0);
        break;
      case L3:
        moveToPosition(m_L3Position_mm);
        break;
      case ALGAE_HIGH:
        moveToPosition(m_L3Position_mm);
        break;
      case ALGAE_LOW:
        moveToPosition(120);
        break;
      case START:
        moveToPosition(0);
        break;
      case LIFT_CLIMB_UP:
     //   moveToPosition(416);
        break;
      case LIFT_CLIMB_DOWN:
        moveToPosition(100);
        break;
      default:
        break;
    }
  }

  public void moveToPosition(double _pos_mm) {
    double volts = m_liftPID.calculate(getPosition_mm(), _pos_mm);
    volts = MathUtil.clamp(volts, m_maxLiftDownVolts, m_maxLiftUpVolts);
    moveWithVoltage(volts);
  }

  public double getPosition_mm() {
    return m_liftMotor.getPosition().getValueAsDouble() / g.LIFT.MOTOR_ROTATIONS_TO_LIFT_DISTANCE_rotPmm;
  }

  public void moveWithVoltage(double _volts) {
    m_liftMotor.setControl(m_liftVoltageOut.withOutput(_volts));

  }
  public void moveWithVoltage(double _volts, CoralLiftState _state) {
    g.CORALLIFT.state = _state;
    if(_volts > 0 && getPosition_mm() < m_L3Position_mm) {
      m_liftMotor.setControl(m_liftVoltageOut.withOutput(_volts));
    }else{
      m_liftMotor.setControl(m_liftVoltageOut.withOutput(0));
    }
    

  }
  public void rotateToAngle(double _angle_deg) {
    double pid = m_rotatePID.calculate(Math.toRadians(getRotateAngle_deg()), Math.toRadians(_angle_deg));
    SmartDashboard.putNumber("Rotate Expected Angle", _angle_deg);
    pid = MathUtil.clamp(pid, -8, 8);
    SmartDashboard.putNumber("RotatePID_volts", pid);
    m_rotateMotor.setVoltage(pid);
  }

  private double getRotateAngle_deg() {
    return m_rotateMotor.getEncoder().getPosition() * 360 / g.CORAL.ROTATE_GEAR_RATIO;
  }
  public boolean isCoralLiftAtSetPoint() {
    if(m_liftPID.atSetpoint()) {
      return true;
    }
    return false;
  }
  public void spin(double _speed) {
    double volts = _speed * g.ROBOT.MAX_BATTERY_SUPPLY_volts;
    m_leftMotor.setControl(m_leftVoltageOut.withOutput(volts));
    m_rightMotor.setControl(m_rightVoltageOut.withOutput(-volts));
  }

  private void configureMotors() {
    TalonFXSConfiguration tfxsConfigure = new TalonFXSConfiguration();
    tfxsConfigure.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;

    MotorOutputConfigs spinnerMotorOutConfig = new MotorOutputConfigs();
    spinnerMotorOutConfig.NeutralMode = NeutralModeValue.Brake;

    m_leftMotor.getConfigurator().apply(tfxsConfigure);
    m_rightMotor.getConfigurator().apply(tfxsConfigure);

    m_leftMotor.getConfigurator().apply(spinnerMotorOutConfig);
    m_rightMotor.getConfigurator().apply(spinnerMotorOutConfig);

    SparkMaxConfig maxConfig = new SparkMaxConfig();
    maxConfig.idleMode(SparkMaxConfig.IdleMode.kBrake);
    m_rotateMotor.configure(maxConfig, SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);

    MotorOutputConfigs liftMotorOutputConfig = new MotorOutputConfigs(); // Creates new MotorOutputConfigs.
    liftMotorOutputConfig.NeutralMode = NeutralModeValue.Brake;
    liftMotorOutputConfig.Inverted = InvertedValue.Clockwise_Positive;
    m_liftMotor.getConfigurator().apply(liftMotorOutputConfig);
  }

  @Override
  public void periodic() {

  }

  @Override
  public void updateDashboard() {
    SmartDashboard.putString("CoralLift/CL_State", g.CORALLIFT.state.toString());
    SmartDashboard.putNumber("CoralLift/Arm Angle", getRotateAngle_deg());
    SmartDashboard.putNumber("CoralLift/Lift Distance mm", getPosition_mm());
    SmartDashboard.putData("CoralLift/CoralLiftSubsystem", this);
  }
}
