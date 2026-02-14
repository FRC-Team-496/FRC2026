// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
//fully importing com file odes not resolve object initialization errors; find com file and confirm it exists

//import edu.kauailabs.navx.frc.AHRS;
//import //com.studica.frc.*;

import edu.wpi.first.wpilibj2.command.*;

import java.util.ArrayList;
import java.util.Map;
import java.util.Set;

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

import java.util.function.BooleanSupplier;


import java.math.*;;



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
  private SendableChooser<Integer> m_chooser = new SendableChooser<Integer>(); 
  private int autoScheduler = 0;


  Thread m_visionThread;
  GenericHID m_driverController = new GenericHID(OIConstants.kDriverControllerPort);
  GenericHID m_driverController2 = new GenericHID(OIConstants.kDriverControllerPort2);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // m_pixy.init();

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
        

      

        //m_climbers.setDefaultCommand(new RunCommand(() -> m_climbers.stop(), m_climbers));        
        // m_gyro.setDefaultCommand(
        //     new RunCommand(
        //     () -> m_gyro.putGyro()    
        //     , m_gyro)
        // );
        m_camera.setDefaultCommand(
            new RunCommand(
            () -> m_camera.startCamera()    
            , m_camera)
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
          //Camera camera = new Camera();


            /*new JoystickButton(m_driverController, 1) 
            .onTrue(new rotate(m_robotDrive, 1, 1));*/

            new JoystickButton(m_driverController, 1)
            .onTrue(new SequentialCommandGroup(new targetArea(),new RunCommand(() -> m_robotDrive.drive(m_camera.getAreaDistance(1, m_camera.getBestYaw()).get(1), m_camera.getAreaDistance(1, m_camera.getBestYaw()).get(0),0, false,.95 ))));


            //new JoystickButton(m_driverController, 2)
            //.onTrue(new InstantCommand(() -> System.out.println("Command Run")));


            //Math.clamp(m_camera.alignTag(1,.3).get(3).doubleValue(), 1.0, -1.0))

            new JoystickButton(m_driverController, 2) //new LimelightAlignment(m_camera)
            .onTrue(new ConditionalCommand(new SequentialCommandGroup(new lineUpToCenter(1), new LimelightDrive(1,"a",1), new lineUpToCenter(1)), new InstantCommand(() -> System.out.println("false")), () -> m_camera.getDetected_ID()==1)
            );
              

            /*new JoystickButton(m_driverController, 3)
            .onTrue(new moveStraight(m_robotDrive , 1, 1));*/
            //this works

           /* new JoystickButton(m_driverController, 3)
            .whileTrue(Commands.defer(
              () -> Commands.select(Map.of(
                1, new SequentialCommandGroup(new lineUpToCenter(), new LimelightDrive(1,"a",1), new lineUpToCenter()))
                , () -> m_camera.getDetected_ID()), Set.of(m_robotDrive)
              ));*/
              

            

            

            

  }








public class moveStraight extends Command{
    double startX;
    DriveSubsystem m_robotDrive;
    private double distance; // in meters

    private int direction;

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
      
      m_robotDrive.drive(.6 * direction, 0.0, 0.0, false, .3); //-.02
    }

    @Override
    public boolean isFinished() {
        return Math.abs(m_robotDrive.getPose().getX() - startX) > (distance );  
    }
}
    
public class targetArea extends Command{
  private boolean done;
  public targetArea(){
    addRequirements(m_robotDrive);
    }

  @Override
  public void execute() {
      done = m_camera.getBestTargetArea(m_camera.getBestYaw());
  }

  @Override
  public boolean isFinished() {
      return m_camera.getDetected_ID()!=1 || done;  
  }
}


/*public class checkTag implements Runnable{
  @Override
  public void run(){
    if(m_camera.getDetected_ID()==16){
      new SequentialCommandGroup(new lineUpToCenter(), new LimelightDrive(1,"a",9), new lineUpToCenter());
    }
    else if(m_camera.getDetected_ID()==9){
      new SequentialCommandGroup(new RunCommand(()-> m_robotDrive.drive(0,0,-m_camera.getBestYaw(),false,.5)), new LimelightDrive(.5,"z",16), new lineUpToCenter());
    }
  }
}*/


  

  public class rotate extends Command{
    DriveSubsystem m_robotDrive;
    private double degrees; 
    private double startRot;
    private int direction;





    public rotate(DriveSubsystem m_robotDrive, double degrees, int direction){
      this.m_robotDrive = m_robotDrive;
      this.degrees = degrees;
      this.direction = direction;
      addRequirements(m_robotDrive);
    }


    @Override
    public void initialize() {
      startRot = m_gyro.yaw();
    }

    @Override
    public void execute() {
      System.out.println(m_robotDrive.getHeading());
      if (degrees !=0.0){
        m_robotDrive.drive(0, 0.0, 1 * direction, false, .3); //-.02
      }
      
    }

    @Override
    public boolean isFinished() {
        return Math.abs(m_gyro.yaw() - startRot) > (degrees);  
    }

  }



public class LimelightDrive extends Command{

    private double pose;
    private double distance;
    private String orientation;
    private double tag;


    public LimelightDrive(double distance, String orientation, double tag){
        this.distance = distance;
        this.orientation = orientation;
        this.tag= tag;
        addRequirements(m_robotDrive);

        // if issue, change camera.... to just m_robotdrive like others
    }

    @Override
    public void initialize() {
        
        pose =m_camera.alignTag(m_camera.getDetected_ID(),distance,orientation);
    }

    @Override
    public void execute() {
        if (m_camera.getDetected_ID() == tag){
          if(orientation.equals("a")){
            if((1-(m_camera.getResultantDistance(m_camera.getDistXFromTag(),m_camera.getDistZFromTag())))<.05){
              m_camera.getDriveSubsystem().drive(.6,0,0,false,.5);}
            else{
              m_camera.getDriveSubsystem().drive(-.6,0,0,false,.5);
            }
        }
          if(orientation.equals("z")){
            m_camera.getDriveSubsystem().drive(0,.8,0,false,.5);
        }
          }
      }
    

    @Override
    public boolean isFinished() {
        return m_camera.getDetected_ID()!=tag || Math.abs(1 - m_camera.getResultantDistance(m_camera.getDistXFromTag(), m_camera.getDistZFromTag())) < .09;  
    }
  }

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
      SmartDashboard.putNumber("lining up", currentX);
      if (currentX>0){
        SmartDashboard.putNumber("lining up >0", currentX);
        m_robotDrive.drive(0, 0, -1, false, .8);
      }
      else{
        SmartDashboard.putNumber("lining up <0", currentX);
        m_robotDrive.drive(0,0,1,false,.8);
      }
    }
  }

  @Override
  public boolean isFinished() {
    return m_camera.getDetected_ID()!=tag || Math.abs(1 - currentX) < 1;  
  }

}

  
















  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
 


  public void teloPeriodic(){
       m_gyro.putGyro();
  }

  int state = 0;
  double mode;
  int intmode;
  double startTime = System.currentTimeMillis();
  

  public void teleopInit(){
      System.out.println("Start");
  }

  public void autoInnit(){
    state=0;
    mode = SmartDashboard.getNumber("Autonomous Mode", 1.0);
    System.out.println(mode);
    intmode = (int) mode;
    startTime = System.currentTimeMillis();
    CommandScheduler.getInstance().schedule(new moveStraight(m_robotDrive, 1.0, 1));



   

  }



  public void autonomousPeriodic(){
    
    
      
    

     
    
  }
  }



