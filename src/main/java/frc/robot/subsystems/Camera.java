package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

//This class has all of the methods to get values from a limelight
public class Camera extends SubsystemBase {
    private static double statX;
    private static double statYaw;
    private static double statY;
    private static int detected_ID;
    private DriveSubsystem driveSubsystem;
    private double[] tagToCamera;
    
    private NetworkTable armTable;
    NetworkTableEntry tx;
    NetworkTableEntry ty;
    NetworkTableEntry ta;
    NetworkTableEntry botpose;
    private int aprilTagID;

    public Camera(DriveSubsystem driveSubsystem){
        this.driveSubsystem = driveSubsystem;
    }
    
    //gets called once when the robot turns on and the camera starts
    public void startCamera() {
        //accesses the network table and defines initial position values
        armTable = NetworkTableInstance.getDefault().getTable("limelight-shooter");
        tx = armTable.getEntry("tx");
        ty = armTable.getEntry("ty");
        ta = armTable.getEntry("ta");
        botpose = armTable.getEntry("botpose");
        
        //only useful if we try metatag alignment or field based driving
        AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);
    }

    //reads and updates values periodically
    public void limelightPeriodic(){
        aprilTagID =  (int) armTable.getEntry("tid").getDouble(-1);
        detected_ID = aprilTagID;
        double x = tx.getDouble(0.0);
        statX = x;
        double y = ty.getDouble(0.0);
        statY = y;
        double[] bot = botpose.getDoubleArray(new double[6]);
        statYaw = bot[5];
        tagToCamera = armTable.getEntry("targetpose_cameraspace").getDoubleArray(new double[6]);
    }

    //prints out all of the positioning values for an april tag when looking from the camera
    public void cameraReadouts(){
        SmartDashboard.putNumber("tx Tag", tagToCamera[0]);
        SmartDashboard.putNumber("ty Tag", tagToCamera[1]);
        SmartDashboard.putNumber("tz Tag", tagToCamera[2]);
        SmartDashboard.putNumber("pitch Tag", tagToCamera[3]);
        SmartDashboard.putNumber("yaw Tag", tagToCamera[4]);
        SmartDashboard.putNumber("roll Tag", tagToCamera[5]);
        SmartDashboard.putNumber("x crosshair",getXCrosshair());
        SmartDashboard.putNumber("Tag ID", aprilTagID);
    }

    public int getDetected_ID(){
        return detected_ID;
    }

    public DriveSubsystem getDriveSubsystem(){
        return driveSubsystem;
    }

    //gets degree value from crosshairs("How many degrees do we need to turn to line up?")
    public double getXCrosshair(){return statX;}
    public double getYawCrosshair(){return statYaw;}
    public double getYCrosshair(){return statY;}

    //gets distance value, camera is 0,0,0("How far do we need to move left/right to line up?")
    public double getDistX(){return NetworkTableInstance.getDefault().getTable("limelight-shooter").getEntry("targetpose_cameraspace").getDoubleArray(new double[6])[0];}
    public double getDistYaw(){return NetworkTableInstance.getDefault().getTable("limelight-shooter").getEntry("targetpose_cameraspace").getDoubleArray(new double[6])[4];}
    public double getDistZ(){return NetworkTableInstance.getDefault().getTable("limelight-shooter").getEntry("targetpose_cameraspace").getDoubleArray(new double[6])[2];}
    
    //gets distance value, tag is 0,0,0("How far do we need to move left/right to line up?")
    public double getDistZFromTag(){return NetworkTableInstance.getDefault().getTable("limelight-shooter").getEntry("botpose_targetspace").getDoubleArray(new double[6])[2];}
    public double getDistXFromTag(){return NetworkTableInstance.getDefault().getTable("limelight-shooter").getEntry("botpose_targetspace").getDoubleArray(new double [6])[0];}

     //gets degree value
    public double getYawFromTag(){return NetworkTableInstance.getDefault().getTable("limelight-shooter").getEntry("botpose_targetspace").getDoubleArray(new double [6])[4];}

    //gets how much of the area of the tag can be seen by the camera (the bigger it is, the more head-on they are to eachother)
    public double getTargetArea(){return NetworkTableInstance.getDefault().getTable("limelight-shooter").getEntry("ta").getDouble(0);}

    //returns the distance between the camera and the tag 
    public double getResultantDistance(double x, double y){return Math.sqrt(Math.pow(x, 2)+Math.pow(y,2));}
}
