// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  private final XboxController controller = new XboxController(1);

  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";

  private final SparkMax motor1;
  private final SparkMax motor2;
  private final SparkMax motor3;
  private final SparkMaxConfig config;

  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    config = new SparkMaxConfig();
    motor1 = new SparkMax(1, MotorType.kBrushless);
    motor2 = new SparkMax(2, MotorType.kBrushless);
    motor3 = new SparkMax(3, MotorType.kBrushless);

    config.smartCurrentLimit(40).inverted(true);
    config.encoder.velocityConversionFactor(Constants.conv);
    // velocityConversionFactor output unit: rotations per second.
    config.closedLoop
    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    .pidf(0.0002, 0.0, 0.0002, 0.010719, ClosedLoopSlot.kSlot0);
    
    motor1.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    motor2.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    motor3.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {}

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
  }
  

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during operator control. */

  private void SetMotorTower(double velocity) {
    motor1.getClosedLoopController().setReference(velocity, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
  }
  private void SetMotorWheel(double velocity) {
    motor2.getClosedLoopController().setReference(velocity, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
  }
  private void SetMotorMaze(double velocity) {
    motor3.getClosedLoopController().setReference(velocity, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
  }
  
  @Override
  public void teleopPeriodic() {
    SmartDashboard.putNumber("Part 1 Motor voltage", motor1.getBusVoltage());

    if(controller.getBButton()){
      SetMotorTower(2.5);
    }else if(controller.getAButton()){
      SetMotorTower(-2.5);
    }else{
      SetMotorTower(0.0);
    }

    SmartDashboard.putNumber("Part 2 Motor voltage", motor2.getBusVoltage());

    SetMotorWheel(controller.getLeftY() * Constants.MOTOR_SPEED );

    if(controller.getLeftY() > 0 ){
      SetMotorWheel(0.0);
    }

    SmartDashboard.putNumber("Part 3 Motor voltage", motor3.getBusVoltage());


    if(controller.getXButton()) {
      SetMotorMaze(2.0);
    }else if(controller.getYButton()) {
      SetMotorMaze(-2.0); 
    }else{
      SetMotorMaze(0.0);
    }
    // double x = controller.getLeftX();
    // double y = -controller.getLeftY();

    // double magnitude = Math.sqrt(x * x + y* y);

    // if(magnitude < Constants.DeadBand) {
      // SetMotorMaze(0.0);
    // }else{
      // double diretion = Math.atan2(y, x);
      // SetMotorMaze(2.0 * diretion);

  }

  /** This function is called once when the robot is disabled. */
    @Override
    public void disabledInit() {}

    /** This function is called periodically when disabled. */
    @Override
    public void disabledPeriodic() {}

    /** This function is called once when test mode is enabled. */
    @Override
    public void testInit() {}

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {}

    /** This function is called once when the robot is first started up. */
    @Override
    public void simulationInit() {}

    /** This function is called periodically whilst in simulation. */
    @Override
    public void simulationPeriodic() {}
}
