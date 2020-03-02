/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Joystick;

//import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import frc.robot.Auto.Timer;
import frc.robot.Subsystems.PIDController;
import frc.robot.Auto.PathPlanner;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */

public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private final SendableChooser<String> chooser = new SendableChooser<>();

  private Joystick driveController;

  private TalonFX leftFront;
  private TalonFX leftBack;
  private TalonFX rightFront;
  private TalonFX rightBack;

  private TalonFXConfiguration configs;

  private PigeonIMU pidgey;

  private double targetAngle;
  private double currentYawPos;

  private Timer timer = new Timer();

  private static double accumulator = 0.0;
  private static int counter = 0;

  public double[] yprArr;

  private boolean isAutoFinished = false; // auto flag for spline

  private double totalTime = 5; // seconds
  private double timeStep = 0.1; // period of control loop on motor controller, seconds
  private double robotTrackWidth = 2; // distance between left and right wheels, feet

  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;

  double[][] straightLine = new double[][] { 
    { 0, 0 }, 
    { 4, 0 }, 
    { 4, 4 },
    { 8, 4 } 
  };

  private final PathPlanner path = new PathPlanner(straightLine);

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {

    // motor init
    rightFront = new TalonFX(1);
    rightBack = new TalonFX(2);
    leftFront = new TalonFX(3);
    leftBack = new TalonFX(4);

    // setting followers and leaders
    rightBack.set(ControlMode.Follower, rightFront.getDeviceID());
    leftBack.set(ControlMode.Follower, leftFront.getDeviceID());

    // to right the right side
    rightFront.setInverted(true);
    rightBack.setInverted(true);

    // ramps to full speed (sec)
    rightFront.configClosedloopRamp(0.5);
    rightBack.configClosedloopRamp(0.5);
    leftFront.configClosedloopRamp(0.5);
    leftBack.configClosedloopRamp(0.5);

    // to stop slippage
    leftFront.setNeutralMode(NeutralMode.Brake);
    leftBack.setNeutralMode(NeutralMode.Brake);
    rightFront.setNeutralMode(NeutralMode.Brake);
    rightBack.setNeutralMode(NeutralMode.Brake);

    //                                   enabled, limit(amp), trigger threshold(amp), trigger threshold time (sec)
    rightFront.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 30, 30, 0.1));
    rightBack.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 30, 30, 0.1));
    leftFront.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 30, 30, 0.1));
    leftBack.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 30, 30, 0.1));

    // pidgeon IMU init
    pidgey = new PigeonIMU(17);
    yprArr = new double[3];
    pidgey.configFactoryDefault();

    // configs pidgey yaw to remote 1 slot
    rightFront.configRemoteFeedbackFilter(pidgey.getDeviceID(), RemoteSensorSource.Pigeon_Yaw, PIDController.REMOTE_1,
        30);

    // configs remote sensor to first PID on right front talon
    rightFront.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor1, PIDController.PIDTurn, 30);

    // configs remote 1 to pid slot 1 on right front talon
    rightFront.selectProfileSlot(PIDController.REMOTE_1, PIDController.PIDTurn);
    leftFront.selectProfileSlot(PIDController.REMOTE_1, PIDController.PIDTurn);

    // config pid values for pigeon
    rightFront.config_kP(1, PIDController.turning.kP);
    rightFront.config_kI(1, PIDController.turning.kI);
    rightFront.config_kD(1, PIDController.turning.kD);
    rightFront.config_kF(1, PIDController.turning.kF);
    rightFront.config_IntegralZone(1, PIDController.turning.kIzone);

    // zero pidgey
    pidgey.setYaw(0, 30);
    pidgey.setAccumZAngle(0, 30);

    // integrated encoder values
    configs = new TalonFXConfiguration();
    configs.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
    leftFront.configAllSettings(configs);
    rightFront.configAllSettings(configs);

    // encoder values to pid 0 values
    rightFront.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, PIDController.PIDPrimary, 30);
    leftFront.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, PIDController.PIDPrimary, 30);

    // sets allowable error for pid 0
    rightFront.configAllowableClosedloopError(PIDController.PIDPrimary, 0, 30);
    leftFront.configAllowableClosedloopError(PIDController.PIDPrimary, 0, 30);

    // configs pid 0 slot to pid 0 on talons
    rightFront.selectProfileSlot(PIDController.REMOTE_0, PIDController.PIDPrimary);
    leftFront.selectProfileSlot(PIDController.REMOTE_0, PIDController.PIDPrimary);

    // configs pid values for straight shot
    rightFront.config_kP(0, PIDController.velocity.kP);
    rightFront.config_kI(0, PIDController.velocity.kI);
    rightFront.config_kD(0, PIDController.velocity.kD);
    rightFront.config_kF(0, PIDController.velocity.kF);
    rightFront.config_IntegralZone(0, PIDController.velocity.kIzone);

    leftFront.config_kP(0, PIDController.velocity.kP);
    leftFront.config_kI(0, PIDController.velocity.kI);
    leftFront.config_kD(0, PIDController.velocity.kD);
    leftFront.config_kF(0, PIDController.velocity.kF);
    leftFront.config_IntegralZone(0, PIDController.velocity.kIzone);

    // sets encoder values to 0
    rightFront.setSelectedSensorPosition(0, PIDController.PIDPrimary, 30);
    leftFront.setSelectedSensorPosition(0, PIDController.PIDPrimary, 30);

    // rightFront.configAuxPIDPolarity(false, 30);

    // initializes drivecontroller
    driveController = new Joystick(0);

  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like diagnostics that you want ran during disabled, autonomous,
   * teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {

    SmartDashboard.putNumber("Left Encoder Velocity", leftFront.getSelectedSensorVelocity()); // to get 100 ms to sec
    SmartDashboard.putNumber("Right Encoder Velocity", rightFront.getSelectedSensorVelocity());
    // velocity in position units per 100 ms

    SmartDashboard.putNumber("left position", leftFront.getSelectedSensorPosition());
    SmartDashboard.putNumber("right Position", rightFront.getSelectedSensorPosition());
    // position units

    SmartDashboard.putNumber("%out", driveController.getY());

    pidgey.getYawPitchRoll(yprArr);
    SmartDashboard.putNumber("Pidgey Yaw", yprArr[0]);

    currentYawPos = rightFront.getSelectedSensorPosition(1);
    SmartDashboard.putNumber("rightFront Yaw", currentYawPos);

    SmartDashboard.putNumber("right current", rightFront.getStatorCurrent());
    SmartDashboard.putNumber("left current", leftFront.getStatorCurrent());

  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable chooser
   * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
   * remove all of the chooser code and uncomment the getString line to get the
   * auto name from the text box below the Gyro
   *
   * <p>
   * You can add additional auto modes by adding additional comparisons to the
   * switch structure below with additional strings. If using the SendableChooser
   * make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {

    // resets counter to 0 w/o reuploading code
    counter = 0;

    timer.reset();

    isAutoFinished = false;

    path.calculate(totalTime, timeStep, robotTrackWidth);

    // zero pidgey
    pidgey.setYaw(0, 30);
    pidgey.setAccumZAngle(0, 30);

  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {

    accumulator += timer.getDT();
    // SmartDashboard.putNumber("acum", accumulator);
    // tester

    SmartDashboard.putNumber("error", rightFront.getClosedLoopError(1));

    if (!isAutoFinished && (accumulator >= path.smoothRightVelocity[counter][0])) {

      // System.out.printf("\n--------------------\nCounter: %d \nLeft Velocity: %f
      // \nRight Velocity: %f",
      // counter, PathPlanner.smoothLeftVelocity[counter][1],
      // PathPlanner.smoothRightVelocity[counter][1]);

      targetAngle = 22.7 * path.heading[counter][1];
      // System.out.println("heading: " + targetAngle);

      rightFront.set(ControlMode.Velocity, path.smoothRightVelocity[counter][1], DemandType.AuxPID, targetAngle);
      leftFront.set(ControlMode.Velocity, path.smoothLeftVelocity[counter][1], DemandType.AuxPID, targetAngle);

      counter++;

      if (counter >= path.smoothLeftVelocity.length) {
        isAutoFinished = true;

        rightFront.set(ControlMode.Velocity, 0); // voltage for a harder stop
        leftFront.set(ControlMode.Velocity, 0);

      }
    }

    timer.update();

  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {

    double forward = -1 * driveController.getY();
    double turn = driveController.getRawAxis(4);
    forward = Deadband(forward);
    turn = Deadband(turn);

    leftFront.set(ControlMode.PercentOutput, forward, DemandType.ArbitraryFeedForward, turn);
    rightFront.set(ControlMode.PercentOutput, forward, DemandType.ArbitraryFeedForward, -turn);

  }

  double Deadband(double value) {
    /* Upper deadband */
    if (value >= +0.05)
      return value;

    /* Lower deadband */
    if (value <= -0.05)
      return value;

    /* Outside deadband */
    return 0;
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}