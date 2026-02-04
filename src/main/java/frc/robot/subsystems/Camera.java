package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import frc.robot.LimelightHelpers;
//import frc.robot.LimelightHelpers.RawFiducial;
//import frc.robot.RobotContainer.moveStraight;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
//import edu.wpi.first.math.geometry.Pose2d;
//import edu.wpi.first.math.geometry.Pose3d;
//import edu.wpi.first.math.controller.*;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
//import edu.wpi.first.wpilibj2.command.Command;
//import frc.robot.RobotContainer;
//import frc.robot.RobotContainer.rotate;
//import com.studica.frc.*;
import frc.robot.subsystems.NavX;
//import frc.robot.subsystems.DriveSubsystem;

import java.util.ArrayList;

public class Camera extends SubsystemBase {
    private static double statX;
    private static double statYaw;
    private static double statY;
    private static int detected_ID;
    private DriveSubsystem driveSubsystem;

    private static int cameraPipelineID = 0;
    private static int neuralNetworkpipelineId;

    //need to figure out acutal measurements for these
    private static NavX m_gyro = new NavX();
    public Camera(DriveSubsystem driveSubsystem){
        this.driveSubsystem = driveSubsystem;
    }
    
    public void startCamera() {
        NetworkTable armTable = NetworkTableInstance.getDefault().getTable("limelight-shooter");
        NetworkTableEntry tx = armTable.getEntry("tx");
        NetworkTableEntry ty = armTable.getEntry("ty");
        NetworkTableEntry ta = armTable.getEntry("ta");
        NetworkTableEntry botpose = armTable.getEntry("botpose");
        NetworkTableEntry pipeline = armTable.getEntry("pipeline");
        int aprilTagID =  (int) armTable.getEntry("tid").getDouble(-1);
        detected_ID = aprilTagID;

        AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);
        
        SmartDashboard.putNumber("tag ID", aprilTagID);
        //read values periodically
        double x = tx.getDouble(0.0);
        statX = x;
        double y = ty.getDouble(0.0);
        statY = y;
        double area = ta.getDouble(0.0);
        double[] bot = botpose.getDoubleArray(new double[6]);
        statYaw = bot[5];



        double[] tagToCamera = armTable.getEntry("targetpose_cameraspace").getDoubleArray(new double[6]);

        SmartDashboard.putNumber("tx Tag", tagToCamera[0]);
        SmartDashboard.putNumber("ty Tag", tagToCamera[1]);
        SmartDashboard.putNumber("tz Tag", tagToCamera[2]);
        SmartDashboard.putNumber("pitch Tag", tagToCamera[3]);
        SmartDashboard.putNumber("yaw Tag", tagToCamera[4]);
        SmartDashboard.putNumber("roll Tag", tagToCamera[5]);
        
        SmartDashboard.putNumber("Tag ID", aprilTagID);
        ArrayList<Double> targetMarkers = alignTag(1,.5, -45,45);

        //getBestTargetArea(getBestYaw());

    }


    public static int getDetected_ID(){
        //System.out.println(detected_ID);
        return detected_ID;
    }

    public DriveSubsystem getDriveSubsystem(){
        return driveSubsystem;
    }

    //gets degree value from crosshairs("How many degrees do we need to turn to line up?")
    public static double getX(){
        return statX;
    }

    public static double getYaw(){
        return statYaw;
    }

    public static double getY(){
        return statY;
    }

    //gets distance value, camera is 0,0,0("How far do we need to move left/right to line up?")
    public static double getDistX(){
        return NetworkTableInstance.getDefault().getTable("limelight-shooter").getEntry("targetpose_cameraspace").getDoubleArray(new double[6])[0];
    }

    public static double getDistYaw(){
        return NetworkTableInstance.getDefault().getTable("limelight-shooter").getEntry("targetpose_cameraspace").getDoubleArray(new double[6])[4];
    }

    public static double getDistZ(){
        return NetworkTableInstance.getDefault().getTable("limelight-shooter").getEntry("targetpose_cameraspace").getDoubleArray(new double[6])[2];
    }
    
    //gets distance value, tag is 0,0,0("How far do we need to move left/right to line up?")
    public static double getDistZFromTag(){
        return NetworkTableInstance.getDefault().getTable("limelight-shooter").getEntry("botpose_targetspace").getDoubleArray(new double[6])[2];
    }

    public static double getDistXFromTag(){
        return NetworkTableInstance.getDefault().getTable("limelight-shooter").getEntry("botpose_targetspace").getDoubleArray(new double [6])[0];
    }
     //gets degree value
    public static double getYawFromTag(){
        return NetworkTableInstance.getDefault().getTable("limelight-shooter").getEntry("botpose_targetspace").getDoubleArray(new double [6])[4];
    }

    public static double getTargetArea(){
        return NetworkTableInstance.getDefault().getTable("limelight-shooter").getEntry("ta").getDouble(0);
    }


    public static double getBestYaw(){
        //create something that checks whether the gryo yaw is calibrated and if it matches the limelight yaw
        //use both to get a more accurate value
        double gyroYaw = m_gyro.yaw();
        double limelightYaw = getYawFromTag();
        double bestYaw = limelightYaw;
        return bestYaw;
    }

    public double getResultantDistance(double x, double y){
        return Math.sqrt(Math.pow(x, 2)+Math.pow(y,2));
    }


    //trig function that determines how far and at what angle to robot needs to go to get to a target position
    public ArrayList<Double> alignTag(double tagID, double distanceFromTag, double minYaw, double maxYaw){


        double currentYawPosition = getBestYaw();
        double yawTarget;
        //angle until centered with tag
        if (currentYawPosition < minYaw){
            double yawChange = currentYawPosition-minYaw;
            yawTarget = minYaw;
        }
        else if(currentYawPosition > maxYaw){
            double yawChange =currentYawPosition - maxYaw;
            yawTarget = maxYaw;
        }
        else{
            yawTarget = currentYawPosition;
        }

        //find x and z target values
        double zTarget = distanceFromTag * Math.sin(yawTarget*(Math.PI/180));
        double xTarget = distanceFromTag * Math.cos(yawTarget*(Math.PI/180));
        ArrayList<Double> returnStatement = new ArrayList<Double>();


        double zPosition = getDistZFromTag();
        double xPosition = getDistXFromTag();
        double yawPosition = getBestYaw();

        //was all addition, change back if needed
        double xPositionChange = xTarget-xPosition;
        double zPositionChange = zPosition-zTarget;
        double yawPositionChange = yawTarget - yawPosition;

        double totalDistanceToMove = getResultantDistance(xPositionChange, zPositionChange);
        double totalYaw = Math.atan2(xPositionChange,-zPositionChange);
        totalYaw = totalYaw*(180/Math.PI);
        totalYaw = totalYaw - yawPosition; 
        //double gyroYaw = m_gyro.yaw();
        //gyroYaw = totalYaw - gyroYaw;
        //should check differences between gyro and limelight yaw - which is closer to expected value
        //numbers are based off of a hub angle of 35 and current yaw of 40
        SmartDashboard.putNumber("X Position: ",xPosition); //should be .5
        SmartDashboard.putNumber("Z position: " ,zPosition);//should be 1.5
        SmartDashboard.putNumber("Camera Yaw: ",yawPosition);//should be 40
        SmartDashboard.putNumber("X Position change: ",xPositionChange);//should be .3
        SmartDashboard.putNumber("Z position change: " ,zPositionChange);//should be .93
        SmartDashboard.putNumber("Camera Yaw position change: " ,yawPositionChange);//should be -15
        SmartDashboard.putNumber("Total Yaw",totalYaw);
        //SmartDashboard.putNumber("Gyro Yaw position change: ");//should be -15 too
        SmartDashboard.putNumber("Distance to move: ", totalDistanceToMove);//should be .98
        SmartDashboard.putNumber("angle to drive at: ",totalYaw);//should be 35?
        SmartDashboard.putNumber("Target Resultant Distance ", getResultantDistance(xTarget,zTarget));
        SmartDashboard.putNumber("Current distance: ",getResultantDistance(getDistXFromTag(),getDistZFromTag()));
        ArrayList<Double> move = new ArrayList<Double>();
        move.add(xPositionChange);
        SmartDashboard.putNumber("X Position Change",xPositionChange);
        move.add(zPositionChange);
        SmartDashboard.putNumber("Z Position Change", -zPosition);
        SmartDashboard.putNumber("Z Position Change", zTarget);
        move.add(yawPositionChange);
        if (yawPositionChange > 0){
            move.add(1.0);
        }
        else{
            move.add(-1.0);
        }
        
        return move;


    }
        


    public boolean getBestTargetArea(double currentYaw){
        SmartDashboard.putString("Target Area: ","running");
        double taCurrent = getTargetArea();
        double taTest=0;
        if (currentYaw>0){
            while (true){
                SmartDashboard.putNumber("Current Area (clockwise): ",taTest);
                driveSubsystem.drive(0, 0.0,-.1, false, .1);
                taTest = getTargetArea();
                if(taTest>taCurrent){
                    taCurrent = taTest;
                }
                else{
                    driveSubsystem.drive(0, 0.0,.1, false, .1);
                    SmartDashboard.putString("done","turning clockwise");
                    break;
                }
            }
        }
        else if (currentYaw<0){
            while (true){
                SmartDashboard.putNumber("Current Area (counterclockwise): ",taTest);
                driveSubsystem.drive(0, 0.0,.1, false, .1);
                taTest = getTargetArea();
                if(taTest>taCurrent){
                    taCurrent = taTest;
                }
                else{
                    driveSubsystem.drive(0, 0.0,-.1, false, .1);
                    SmartDashboard.putString("done","turning counterclockwise");
                    break;
                }
            }
        }
        return true;
    }

    public ArrayList<Double> getAreaDistance(double distance, double yaw){
        ArrayList<Double> returnStatement = new ArrayList<Double>();
        double x = distance * Math.cos(yaw);
        double z = distance *Math.sin(yaw);
        double currentX = getDistXFromTag();
        double currentZ = getDistZFromTag()*-1;
        x = currentX-x;
        z= currentZ-z;
        returnStatement.add(x);
        returnStatement.add(z);
        return returnStatement;
    }


}
