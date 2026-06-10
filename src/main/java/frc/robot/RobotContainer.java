// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//fully importing com file odes not resolve object initialization errors; find com file and confirm it exist
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Camera;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.NavX;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.Components.*;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final Camera m_camera = new Camera(m_robotDrive);
  private NavX m_gyro = new NavX();
  private final SendableChooser<Command> autoChooser; 
  private final shooter m_shooter = new shooter();
  private final Belt m_belt = new Belt();
  private final Intake m_intake = new Intake();
  Thread m_visionThread;
  GenericHID m_driverController = new GenericHID(OIConstants.kDriverControllerPort);
  GenericHID m_driverController2 = new GenericHID(OIConstants.kDriverControllerPort2);
              

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    m_camera.startCamera();

    NamedCommands.registerCommand("Toggle Intake", new InstantCommand(() -> m_intake.toggle()));
    NamedCommands.registerCommand("Toggle Shooter", new ParallelCommandGroup(
      new InstantCommand(() -> m_shooter.toggle()),
      new InstantCommand(() -> m_belt.toggle())

    ));
    NamedCommands.registerCommand("Line up Red", new SequentialCommandGroup(new lineUpToCenter(10),new LimelightDrive(1.9,10), new lineUpToCenter(10)));
    NamedCommands.registerCommand("Line up Blue", new SequentialCommandGroup(new lineUpToCenter(26),new LimelightDrive(1.9,26), new lineUpToCenter(26)));
    
    autoChooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData("Auto Chooser", autoChooser);

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                MathUtil.applyDeadband(-m_driverController.getRawAxis(1), 0.2), 
                MathUtil.applyDeadband(-m_driverController.getRawAxis(0), 0.2),
                MathUtil.applyDeadband(-m_driverController.getRawAxis(2), 0.2),
                false, (m_driverController.getRawAxis(3)+1)/2),
            m_robotDrive)
            );
        }
        

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
  // Driving, joystick on driver controller
    new JoystickButton(m_driverController, Button.kR1.value)
      .whileTrue(new RunCommand(
        () -> m_robotDrive.setX(),
          m_robotDrive));    

    //makes buttons do stuff
    new JoystickButton(m_driverController2, 1)
      .onTrue(new ParallelCommandGroup(
        new InstantCommand(() -> m_shooter.toggle()), 
        new InstantCommand(() -> m_belt.toggle())
      ));

    new JoystickButton(m_driverController2, 3)
      .onTrue(new ParallelCommandGroup(
        new InstantCommand(() -> m_shooter.reverse()), 
        new InstantCommand(() -> m_belt.reverse())
      ));

    new JoystickButton(m_driverController, 1)
      .onTrue(new InstantCommand(() -> m_intake.toggle()));

      //this was the lineup sequence to shoot 
      new JoystickButton(m_driverController2, 2) 
        .onTrue(new ConditionalCommand(new SequentialCommandGroup(new lineUpToCenter(10),new LimelightDrive(2.05,10), new lineUpToCenter(10)), new ConditionalCommand(
          new SequentialCommandGroup(new lineUpToCenter(26),new LimelightDrive(2.05,26), new lineUpToCenter(26)), 
          new InstantCommand(() -> System.out.println("False")),
          () -> m_camera.getDetected_ID()==26),
          () -> m_camera.getDetected_ID()==10 
        ));

      new JoystickButton(m_driverController, 5)
      .onTrue(new LimelightDrive(1,5));
  }

//moves in a straight line for a certain distance
public class moveStraight extends Command{
    double startX;
    DriveSubsystem m_robotDrive;
    private double distance; // in meters ("give or take a smidge" - ruhoy), momentum seems to take it slightly further than the measurement you give
    private int direction; //1 is forward

    public moveStraight(DriveSubsystem m_robotDrive, double distance, int direction){
      this.m_robotDrive = m_robotDrive;
      this.distance = distance;
      this.direction = direction;
      addRequirements(m_robotDrive);
    }

    @Override
    public void initialize() {
      startX = m_robotDrive.getPose().getX();
    }

    @Override
    public void execute() {
      m_robotDrive.drive(.6 * direction, 0.0, 0.0, false, .3); 
    }

    @Override
    public boolean isFinished() {
        return Math.abs(m_robotDrive.getPose().getX() - startX) >= (distance );  
    }

    @Override
    public void end(boolean interrupted){
      m_robotDrive.drive(0,0,0,false,0);
    }
}


//drives until the robot is a certain distance away from a tag or loses the tag
//distance includes how far the camera is into the robot 
//also overshoots a little bit, calling twice can help
public class LimelightDrive extends Command{
    private double distance;
    private double tag;

    public LimelightDrive(double distance, double tag){
        this.distance = distance;
        this.tag= tag;
        addRequirements(m_robotDrive);
    }

    @Override
    public void execute() {
        if (m_camera.getDetected_ID() == tag){
          if((distance-(m_camera.getResultantDistance(m_camera.getDistXFromTag(),m_camera.getDistZFromTag())))<.05){
            m_camera.getDriveSubsystem().drive(.6,0,0,false,.5);}
          else{
            m_camera.getDriveSubsystem().drive(-.6,0,0,false,.5);
        }
          }
      }

    @Override
    public boolean isFinished() {
        return m_camera.getDetected_ID() != tag || Math.abs(distance - m_camera.getResultantDistance(m_camera.getDistXFromTag(), m_camera.getDistZFromTag())) < .09;  
    }
  }

//rotates until the robot is facing the tag - works
public class lineUpToCenter extends Command{
  private double currentX;
  private double tag;

  public lineUpToCenter(double tag){
    this.tag = tag;
    addRequirements(m_robotDrive);
  }

  @Override
  public void initialize() {
      currentX = m_camera.getX();
  }

  @Override
  public void execute() {
    currentX = m_camera.getX();
    if (currentX !=0){
      if (currentX>0){
        m_robotDrive.drive(0, 0, -1, false, .8);
      }
      else{
        m_robotDrive.drive(0,0,1,false,.8); 
      }
    }
  }

  @Override
  public boolean isFinished() {
    return m_camera.getDetected_ID() != tag || Math.abs(1 - currentX) < 1;  
  }

}

//rotates clockwise until it sees a specific tag
public class limelightRotate extends Command{
  private double tag;

  public limelightRotate(double tag){
    this.tag = tag;
    addRequirements(m_robotDrive);
  }

  @Override
  public void execute() {
    m_robotDrive.drive(0,0,-1,false,.8);
  }

  @Override
  public boolean isFinished(){
    return m_camera.getDetected_ID() == tag;
  }

  @Override
  public void end(boolean interrupted){
    m_robotDrive.drive(0,0,0,false,0); 
  }
}


//shooter command sequence - delete in new season
public SequentialCommandGroup shoot = new SequentialCommandGroup(

new InstantCommand(() -> System.out.println("iterate")),

new ParallelDeadlineGroup(
  new WaitCommand(4), 
  new InstantCommand(() -> m_shooter.toggle()), 
  new InstantCommand(() -> m_belt.toggle())
  
  ),
new ParallelDeadlineGroup(
  new WaitCommand(8), 
  new InstantCommand(() -> m_shooter.toggle()), 
  new InstantCommand(() -> m_belt.toggle())
  
  ),

new ParallelCommandGroup(
  new InstantCommand(() -> m_shooter.toggle()), 
  new InstantCommand(() -> m_belt.toggle())

)
);

public Command getAutonomousCommand() {
 return autoChooser.getSelected(); //changes depending on what you want to run.
}


//different auto path options
SequentialCommandGroup Auto2 = new SequentialCommandGroup(
        new InstantCommand(() -> System.out.println("iteration")),
        new ParallelDeadlineGroup(new WaitCommand(1.2), new moveStraight(m_robotDrive, 1, -1)),
        shoot

        );


SequentialCommandGroup AutoMiddle = new SequentialCommandGroup(
        new InstantCommand(() -> m_intake.toggle()),
        new ParallelDeadlineGroup(new WaitCommand(1), new moveStraight(m_robotDrive, 1, -1)),

        new ParallelDeadlineGroup(new WaitCommand(2), new InstantCommand(() -> m_robotDrive.drive(1, 0, 0 ,false, .3))),
        new ParallelDeadlineGroup(new WaitCommand(3), new InstantCommand(() -> m_robotDrive.drive(.7, 0, 0 ,false, .3))),

        new InstantCommand(() -> m_shooter.toggle()),
        new InstantCommand(() -> m_belt.toggle()),
        new ParallelDeadlineGroup(new WaitCommand(5.7),  new InstantCommand(() -> m_robotDrive.drive(-.7, 0, 0 ,false, .3))),
        new SequentialCommandGroup(new lineUpToCenter(26),new LimelightDrive(2.05,26), new lineUpToCenter(26)),
        new InstantCommand(() -> System.out.println("aligned")),
                new InstantCommand(() -> m_shooter.toggle()),
        new InstantCommand(() -> m_belt.toggle())   
        );




  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
 

  //calls periodic methods for different subsystems, very important to keep values updated
  public void teloPeriodic(){
      m_gyro.putGyro();
      m_camera.limelightPeriodic();
      m_camera.cameraReadouts();

  }

  int state = 0;
  double mode;
  int intmode;
  double startTime = System.currentTimeMillis();
  
  //sets up subsystems restart at 0 position regardless of where auto ended
  public void teleopInit(){
      System.out.println("Start");
      m_belt.setSequence(0);
      m_shooter.setSequence(0);
  }

  //sets up begining of auto state - ensure its the same every time
  public void autoInnit(){
    state=0;
    mode = SmartDashboard.getNumber("Autonomous Mode", 1.0);
    System.out.println(mode);
    intmode = (int) mode;
    CommandScheduler.getInstance().schedule(Auto2); //picks which auto to use
    switch(intmode){
      case(1):
      case(2):
      case(3):
      case(4):
      case(5):
      case(6):
    }

  }


  //updates values periodically in autonomous, very important to use limelight code in auto
  public void autonomousPeriodic(){
    
    m_camera.limelightPeriodic();
    
    /*
     *   case(2):
     * 
     *   Command scheduler schedules a single sequential command
     * 
     *  First command move back a certain ammount
     * 
     *  second command shoots for however many seconds.
     * 
     * 
     * 
     * 
     * 
     */
  }
  }