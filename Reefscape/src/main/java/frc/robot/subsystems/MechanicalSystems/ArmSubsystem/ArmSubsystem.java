    /**Overview
     * This code robot is to tell the other subsytems how to contol there motors
     * By taken in where the arm should move and comparing it to where it and other part of the arms are
     * this should allow you to have lot of control over how it moves
     * 
     * NOTE: We have never had a sysyem this complex so this code is untest 
     */
    package frc.robot.subsystems.MechanicalSystems.ArmSubsystem;

    import edu.wpi.first.wpilibj2.command.SubsystemBase;
    import frc.robot.Constants.ArmConstants;
    
    public class ArmSubsystem extends SubsystemBase { 
        private ArmPivot pivotSubsystem;
        private ArmExtention extentionSubsystem;
        private ArmWrist wristSubsystem;
        private ArmManipulator manipulatorSubsystem;
    
        public static double DesiredPivotPosition = ArmConstants.Home_Pivot;
        public static double DesiredExtentionPosition = ArmConstants.Home_Extention;
        public static double DesiredWristPosition = ArmConstants.Home_Wrist; 
    
        // Constructor, accepting ArmWrist as a parameter
        public ArmSubsystem(ArmPivot pivotSubsystem,ArmExtention extentionSubsystem,ArmWrist wristSubsystem) {
            this.pivotSubsystem = pivotSubsystem;
            this.extentionSubsystem = extentionSubsystem;
            this.wristSubsystem = wristSubsystem;
            this.manipulatorSubsystem = manipulatorSubsystem;
        }
    
        // Method to get the pivot position from the wrist subsystem
        public double getPivotPosition(){
            return wristSubsystem.getWristPosition();  // Ensure this method is in ArmWrist
        }


    public double getExtentionPosition(){
        return 0.0;
    }
    public double getWristPosition(){
        return 0.0;
    }

    //This method will be called to set the Arm to a DesiredLocation
    public void setDisinatedPosition(int DesiredPosition){
        //Follow the example and duplicated for each state
        if (DesiredPosition == ArmConstants.Home){
            DesiredPivotPosition = ArmConstants.Home_Pivot;
            DesiredExtentionPosition = ArmConstants.Home_Extention;
            DesiredWristPosition = ArmConstants.Home_Wrist; 
        }
        else if(DesiredPosition == ArmConstants.L1){
            DesiredPivotPosition = ArmConstants.L1_Pivot;
            DesiredExtentionPosition = ArmConstants.L1_Extention;
            DesiredWristPosition = ArmConstants.L1_Wrist;}


        //This is an contintion will be called if nonof the obojve methods are called 
        //I would recommend setting this to the current Position of the arm so it will not have if 
        // does not know where it is.
        else{
            DesiredPivotPosition = pivotSubsystem.getPivotPosition();
            DesiredExtentionPosition = extentionSubsystem.getExtentionPosition();
            DesiredWristPosition = wristSubsystem.getWristPosition();
        }
    }



    //These getter will be used to move the arm,
    //TODO extablish a math matical relation ship between the current states pf the arm.
    //Example you do not want the arm to start extrending till the pivot is more the 45 degress. then you want it to 
    //extend as a sin(function)

    public double setDesiredPivotPosition(){
        //wristSubsystem.setWristPosition(DesiredWristPosition);
        return 0.0;
    }
    public double getDesiredExtentionPosition(){
        return 0.0;
    }
    public double getDesiredWristPosition(){
        return DesiredWristPosition;
    }
    
}
