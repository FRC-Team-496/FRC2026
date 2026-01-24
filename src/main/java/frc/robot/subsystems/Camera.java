package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.RawFiducial;
import frc.robot.RobotContainer.moveStraight;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.controller.*;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class Camera extends SubsystemBase {
    private static double statX;
    private static double statYaw;
    private static double statY;
    private static int detected_ID;

    private static int cameraPipelineID = 0;
    private static int neuralNetworkpipelineId;

    private static double[] tagToCamera;

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

        alignTag(1, 1, 2, 30);

    }


    public static int getDetected_ID(){
        //System.out.println(detected_ID);
        return detected_ID;
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
    
    //gets distance value, camera is 0,0,0("How far do we need to move left/right to line up?")
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

    public void alignTag(double tagID, double xTarget, double zTarget, double yawTarget){
        double zPosition = getDistZFromTag();
        double xPosition = getDistXFromTag();
        double yawPosition = getYawFromTag();

        double xPositionChange = xPosition + xTarget;
        double zPositionChange = zPosition+zTarget;
        double yawPositionChange = yawPosition + yawTarget;

        double totalDistanceToMove = Math.sqrt(Math.pow(zPositionChange,2) + Math.pow(xPositionChange, 2) );
        double totalYaw = Math.atan2(xPositionChange,-zPositionChange);
        totalYaw = totalYaw*(180/Math.PI);
        totalYaw = totalYaw - yawPosition; 
        

        SmartDashboard.putNumber("X Position: ",xPosition);
        SmartDashboard.putNumber("Z position: " ,zPosition);
        SmartDashboard.putNumber("Yaw: ",yawPosition);
        SmartDashboard.putNumber("X Position change: ",xPositionChange);
        SmartDashboard.putNumber("Z position change: " ,zPositionChange);
        SmartDashboard.putNumber("Yaw position change: " ,yawPositionChange);
        SmartDashboard.putNumber("Direction to drive in: ", totalDistanceToMove);
        SmartDashboard.putNumber("angle to drive at: ",totalYaw);







        //Pose3d current_position = LimelightHelpers.getBotPose3d_wpiBlue("limelight-shooter");
        //if (tagID == detected_ID){
            //if (xDegrees != 500 && xDistance != 500){
                //while (xDistance != NetworkTableInstance.getDefault().getTable("limelight-shooter").getEntry("targetpose_cameraspace").getDoubleArray(new double[6])[0]){
                    //double distanceToMove = NetworkTableInstance.getDefault().getTable("limelight-shooter").getEntry("targetpose_cameraspace").getDoubleArray(new double[6])[0] - xDistance;
                    //if (distanceToMove>0){
                          //;
                   //}
                    
                //}
            //}
            //while (NetworkTableInstance.getDefault().getTable("limelight-shooter").getEntry("targetpose_cameraspace").getDoubleArray(new double[6])[2]!= distance){
                

        //}
        
    }

    
}
