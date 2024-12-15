// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.sql.DriverAction;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.AutoCommand;
import frc.robot.commands.DropHarvesterCommand;
import frc.robot.commands.ElevatorDownCommand;
import frc.robot.commands.ElevatorRetractStopCommand;
import frc.robot.commands.ElevatorRetractCommand;
import frc.robot.commands.ElevatorUpCommand;
import frc.robot.commands.HarvestOffCommand;
import frc.robot.commands.HarvestOnCommand;
import frc.robot.commands.HarvesterBackwards;
import frc.robot.commands.HarvesterBackwardsStop;
import frc.robot.commands.HarvesterDownCommand;
import frc.robot.commands.HarvesterUpCommand;
import frc.robot.commands.LatchCloseCommand;
import frc.robot.commands.LatchOpenCommand;
import frc.robot.commands.LineUpCommand;
import frc.robot.commands.LineUpStopCommand;
import frc.robot.commands.MovingCommand;
import frc.robot.commands.ShooterIndexBackCommand;
import frc.robot.commands.ShooterIndexStartCommand;
import frc.robot.commands.ShooterIndexStopCommand;
import frc.robot.commands.ShooterStartCommand;
import frc.robot.commands.ShooterStopCommand;
import frc.robot.subsystems.CompressorSub;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Drivetrain2;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Harvester;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj.Joystick;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private final Harvester harvester = new Harvester();
  private final HarvestOnCommand harvestOnCommand = new HarvestOnCommand(harvester);
  private final HarvestOffCommand harvestOffCommand = new HarvestOffCommand(harvester);


  private final HarvesterUpCommand harvesterUpCommand = new HarvesterUpCommand(harvester);
  private final HarvesterDownCommand harvesterDownCommand = new HarvesterDownCommand(harvester);



  private final Elevator elevator = new Elevator();

  //private final LatchCloseCommand latchCloseCommand = new LatchCloseCommand(elevator);
  //private final LatchOpenCommand latchOpenCommand = new LatchOpenCommand(elevator);

  private final ElevatorUpCommand elevatorUpCommand = new ElevatorUpCommand(elevator);
  private final ElevatorDownCommand elevatorDownCommand = new ElevatorDownCommand(elevator);
  private final ElevatorRetractCommand elevatorRetractCommand = new ElevatorRetractCommand(elevator);
  private final ElevatorRetractStopCommand elevatorRetractStopCommand = new ElevatorRetractStopCommand(elevator);

  private final LimeLight limelight = new LimeLight();
  private final Drivetrain2 drivetrain = new Drivetrain2(limelight);
  private final MovingCommand movingCommand = new MovingCommand(drivetrain, limelight);

  private final Shooter shooter = new Shooter();
  private final ShooterStartCommand shooterstart = new ShooterStartCommand(shooter, Constants.shooterSpeed);
  private final ShooterStopCommand shooterstop = new ShooterStopCommand(shooter);

  private final ShooterIndexStartCommand shooterindexstart = new ShooterIndexStartCommand(shooter);
  private final ShooterIndexStopCommand shooterindexstop = new ShooterIndexStopCommand(shooter);
  private final ShooterIndexBackCommand shooterindexback = new ShooterIndexBackCommand(shooter);

  private final HarvesterBackwards harvesterBackwards = new HarvesterBackwards(harvester, shooter);
  private final HarvesterBackwardsStop harvesterBackwardsStop = new HarvesterBackwardsStop(harvester, shooter);
  
  //private final DropHarvesterCommand dropHarvester = new DropHarvesterCommand(harvester);

  // private final AutoCommand m_exampleSubsystem = new ExampleSubsystem();
  private final AutoCommand m_autoCommand = new AutoCommand(drivetrain, shooter, harvester, limelight);

  private final LineUpCommand lineUpCommand = new LineUpCommand(drivetrain, limelight);
  private final LineUpStopCommand lineUpStopCommand = new LineUpStopCommand(drivetrain, limelight);

  private final Joystick joystick1 = new Joystick(0);

  private final XboxController controller1 = new XboxController(1);

  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  public void teleopInit() {
    drivetrain.setDefaultCommand(movingCommand);
  }

  public void autonomousInit() {
    System.out.println("autoinit");
    // drivetrain.setDefaultCommand(null);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    JoystickButton triggerButton = new JoystickButton(joystick1, 1);
    triggerButton.whenPressed(harvestOnCommand);
    triggerButton.whenReleased(harvestOffCommand);

    JoystickButton harvBack = new JoystickButton(joystick1, 5);
    harvBack.whenPressed(harvesterBackwards);
    harvBack.whenReleased(harvesterBackwardsStop);

     JoystickButton HarvUp = new JoystickButton(controller1, 1);
     JoystickButton HarvDown = new JoystickButton(controller1, 2);
     HarvUp.whenPressed(harvesterUpCommand);
     HarvDown.whenPressed(harvesterDownCommand);
    JoystickButton elvUp = new JoystickButton(joystick1, 12);
    JoystickButton elvDown = new JoystickButton(joystick1, 11);
    JoystickButton elvRetract = new JoystickButton(joystick1, 10);
    
    elvUp.whenPressed(elevatorUpCommand);

    elvDown.whenPressed(elevatorDownCommand);

    elvRetract.whenHeld(elevatorRetractCommand);
    elvRetract.whenReleased(elevatorRetractStopCommand);

    JoystickButton lineUpLimelightButton = new JoystickButton(joystick1, 2);
    lineUpLimelightButton.whenPressed(lineUpCommand);
    lineUpLimelightButton.whenReleased(lineUpStopCommand);


    //JoystickButton latchOpen = new JoystickButton(joystick1, 4);
    // JoystickButton latchClose = new JoystickButton(controller1, 2);
    //latchOpen.whenPressed(latchOpenCommand);
    // latchClose.whenPressed(latchCloseCommand);

    // JoystickButton dropHarvesterButton = new JoystickButton(controller1, 10);
    // dropHarvesterButton.whenPressed(dropHarvester);

    JoystickButton LT = new JoystickButton(controller1, 7);
    JoystickButton RT = new JoystickButton(controller1, 8);
    RT.whenPressed(shooterstart);
    LT.whenPressed(shooterstop);

    JoystickButton LB = new JoystickButton(controller1, 5);
    JoystickButton RB = new JoystickButton(controller1, 6);
    RB.whenPressed(shooterindexstart);
    LB.whenPressed(shooterindexback);

    RB.whenReleased(shooterindexstop);
    LB.whenReleased(shooterindexstop);

  }

  public Joystick getJoystick1() {
    return joystick1;
  }

  public Command getAutonomousCommand() {
    System.out.println("getauto");
    return m_autoCommand;
  }
}
