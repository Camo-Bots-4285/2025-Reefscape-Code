/*
Custom code that will help the robot know how to self drive

Goals 
    1) animate driving to help assistdriver
    2) driving to game piece 
    3) use lane logic to navigate know absticals on the feild


System is fully functional if PIDs, lines, and target are given correctly 

 */

package frc.robot.subsystems.Drive;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Set;

//import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.Swerve.MoveToNoteByPose;
import frc.robot.commands.Swerve.TeleopSwerve;
import frc.robot.subsystems.ComputerVision.NoteDetection;

public class SelfDriving extends  SubsystemBase{
   
public static double TargetPoseX;
public static double TargetPoseY;
public static double TargetPoseRotation;

public static double StartingTargetPoseX;
public static double StartingTargetPoseY;
public static double StartingTargetPoseRotation;

public static double PoseDifferenceX;
public static double PoseDifferenceY;
public static double PoseDifferenceRotation;
public static double PoseDifferenceRotationRaw;

public static double XSpeed;
public static double YSpeed;
public static double RotationSpeed;

public static double XSpeedFinal;
public static double YSpeedFinal;
public static double RotationSpeedFinal;

public static boolean CloseEnough;
public static double HowFar;
public static double HowStraght;
public static boolean TranslationClose;
public static boolean RotationClose; 

public static boolean AutoSetUpMoveLeft;
public static boolean AutoSetUpMoveRight;
public static boolean AutoSetUpMoveForward;
public static boolean AutoSetUpMoveBackward;
public static boolean AutoSetUpRotateCounterClockWise;
public static boolean AutoSetUpRotateCloskWise;
public static boolean AutoSetUpCloseEnough;

public static double LaneLogicPoseX;
public static double LaneLogicPoseY;
public static double LaneLogicPoseRotation;
public static boolean LaneLogicFinalDestination = false;

public static boolean UpdatingTarget; 
// public static boolean AutoSetUpRotateCounterClockWise;
// public static boolean AutoSetUpRotateCloskWise;
// public static boolean AutoSetUpCloseEnough;

//Value requested will not go over this value
public static double MaxTranslationSpeed = 1.0;//3.5
public static double MaxRotationSpeed = Math.PI;
public static double Y_X_Ratio = 4;
public static double XCompensationValue = 1; 


//Make the value reqest from the drive ramp the power to get to target
//WARNING this could mess with deceleration if PID decelerates faster then is value
//to fix issue lower this but would prefer for it to be high for smoother motion
//COmment this out when first tuning PIDS
/*Slew Limiter */
static SlewRateLimiter MaxTranslationAccelerationX = new SlewRateLimiter(.55);
static SlewRateLimiter MaxTranslationAccelerationY = new SlewRateLimiter(.55);
static SlewRateLimiter MaxRotationAcceleration = new SlewRateLimiter(2*Math.PI);

// PIDs to Contol Driving to Note
public static PIDController NoteTranslation = new PIDController(0.285, 0, 0.0);
public static PIDController NoteRotation = new PIDController(0.325, 0, 0);

// PIDs to Control Driving to Pose  
public static PIDController MoveToPoseTranslation = new PIDController(0.285, 0.0, 0.0);
public static PIDController MoveToPoseRotation = new PIDController(0.00895, 0, 0);




    //This method will take the current Pose and compare it to the Pose of starting auto and will give variables to the LED 
    //Needs to be Test 
    public void AutoSetUp() {

     
       //If you want to change the order of auto set up change the order in which it checks for error
        if(Math.abs(PoseDifferenceY) > 0.015){
                if(PoseDifferenceY > 0){//Positive Moves to Right
                    AutoSetUpMoveRight = true;
                }
                else{//Negative Moves to Left
                    AutoSetUpMoveLeft = true;
                } 
        }
        else if(Math.abs(PoseDifferenceRotationRaw) > 1){
                if(PoseDifferenceRotationRaw > 0){//Need to double check but i think Positve should rotate counterclock wise
                    AutoSetUpRotateCounterClockWise = true;
                }
                else{//Need to double check but i think Negative should rotate clock wise
                    AutoSetUpRotateCounterClockWise = true;
                }
        }
        else if(Math.abs(PoseDifferenceX) > 0.015){
                if(PoseDifferenceX > 0){//Positive Moves to Backward
                    AutoSetUpMoveBackward = true;
                }
                else{//Negative Moves to 
                    AutoSetUpMoveForward = true;
                }
        }
        else{//Robot is within set distances and is ready for auto
                    AutoSetUpCloseEnough = true;
        }

    }

    //This method will be used to help drive the robot to the game pieces
    public static void DriveCalculationNote() {

    XSpeed = -SelfDriving.NoteTranslation.calculate(NoteDetection.MainXDistance, 0.3);
    RotationSpeed = SelfDriving.NoteRotation.calculate(NoteDetection.MainYDistance, 0.0);

 // Sets a max out put for variable
    XSpeed =  Math.abs(XSpeed) < MaxTranslationSpeed ?  XSpeed : MaxTranslationSpeed;
    YSpeed =  Math.abs(YSpeed) < MaxTranslationSpeed ?  YSpeed : MaxTranslationSpeed;
    RotationSpeed =  Math.abs(RotationSpeed) < MaxRotationSpeed ?  RotationSpeed : MaxRotationSpeed; 

    //Apply the accelertion limiter
    XSpeedFinal =  MaxTranslationAccelerationX.calculate(XSpeed);
    RotationSpeedFinal = MaxRotationAcceleration.calculate(RotationSpeed);

}

/*Added 10/27/2024 has to be tested this will allow for robot to move not in a straight line 
making it easier to make zone*/
// public static Command getAutoDrivingPath(int pathnumber) {

//     if(RobotState.isAutonomous()==true){
//        // return AutoBuilder.buildAuto("StraitLineTest");
//     }
//     else if(RobotState.isAutonomous()==false){

//         if(pathnumber == 1){
//         //return AutoBuilder.buildAuto("StraitLineTest");
//         }
//         if(pathnumber == 2){
//         // return AutoBuilder.buildAuto("StraitLineTest");
//         }
//         if(pathnumber == 3){
//         //return AutoBuilder.buildAuto("StraitLineTest");
//         }
//         else{return AutoBuilder.buildAuto("StraitLineTest");}
//     }
//     else{ return AutoBuilder.buildAuto("StraitLineTest");}

// }






//The following method sets the target for the self driving systems 
//Example of calling with a button: btn_self_driving_shoot.whileTrue(new RunCommand(() -> m_selfDriving.setTargetPose(1)));
public void setTargetPose(int set) {
if (set == 0){}

//Shotting Infront of Speacker
if (set == 1){ 
    //Pose and Angle For Red
    if (Constants.isRed == true){
    TargetPoseX = 15.0;
    TargetPoseY = 5.55;
    TargetPoseRotation = 0;
        }
    //Pose and Angle For Blue
    if (Constants.isRed == false){
    TargetPoseX = 1.38;
    TargetPoseY = 5.55;
    TargetPoseRotation = 0;
        }
    }
//Amp Pose 
if (set == 2){ 
    //Pose and Angle For Red
    if (Constants.isRed == true){
    TargetPoseX = 14.7;
    TargetPoseY = 7.6;
    TargetPoseRotation = 90;
        }
    //Pose and Angle For Blue
    if (Constants.isRed == false){
    TargetPoseX = 1.80;
    TargetPoseY = 7.60;
    TargetPoseRotation = 90;
        }

    }

    //Note Pose 
        if (set == 3){

            TargetPoseX = NoteDetection.MainNoteXRobot + SwerveBasePose.currentPoseX;
            TargetPoseY = NoteDetection.MainNoteYRobot + SwerveBasePose.currentPoseY;
            // TargetPoseRotation = RobotContainer.angleController.calculate(m_SwerveBasePose.getPose().getRotation().getRadians(), MoveToNoteByPose.NOTE_POSE.getRotation().getRadians());
        }
//Starting position for auto A
        if (set == 4){ 
    //Pose and Angle For Red
    if (Constants.isRed == false){
    TargetPoseX = 14.7;
    TargetPoseY = 7.6;
    TargetPoseRotation = 90;
        }
    //Pose and Angle For Blue
    if (Constants.isRed == true){
    TargetPoseX = 1.80;
    TargetPoseY = 7.60;
    TargetPoseRotation = 90;
        }
    }
    //Call when Lane Logic is active to drive to destination
        if (set == 10){

            TargetPoseX = LaneLogicPoseX;
            TargetPoseY = LaneLogicPoseY;
            TargetPoseRotation = LaneLogicPoseRotation;
        
        }
    }



//The following methed need to be call when selfdriving is used
public static void DriveCalculationPose() {
    //Because we are driving in feild centric for this command the number will change signs depending on allaince
  
   if (Constants.isRed == true){
        PoseDifferenceX = TargetPoseX - SwerveBasePose.currentPoseX;
        PoseDifferenceY = TargetPoseY - SwerveBasePose.currentPoseY;// Need to test might not be right
        PoseDifferenceRotationRaw = SwerveBasePose.currentPoseRotation -TargetPoseRotation; // Need to test might not be right   
    }
        if (Constants.isRed == false){
        PoseDifferenceX = SwerveBasePose.currentPoseX - TargetPoseX;
        PoseDifferenceY = SwerveBasePose.currentPoseY - TargetPoseY;// Need to test might not be right
        PoseDifferenceRotationRaw = SwerveBasePose.currentPoseRotation -TargetPoseRotation;// Need to test might not be right
    }
//Roation should be the same regardless of alliance
//The following will optimize the rotation to go left or right
        if (PoseDifferenceRotationRaw > 180){
        PoseDifferenceRotation = PoseDifferenceRotationRaw - 360;
    }
    //  if (PoseDifferenceRotationRaw < -180){
    //     PoseDifferenceRotation = PoseDifferenceRotationRaw + 360;
    // }
        else {
        PoseDifferenceRotation =  PoseDifferenceRotationRaw;
    }

  
    //Apllies PID to move to Target Pose
    XSpeed =  MoveToPoseTranslation.calculate(PoseDifferenceX, 0.0);
    YSpeed =  MoveToPoseTranslation.calculate(PoseDifferenceY, 0.0);
    RotationSpeed = MoveToPoseRotation.calculate(PoseDifferenceRotation, 0.0);
   
    
    //Sets a max out put for variable
    XSpeed =  Math.abs(XSpeed) < MaxTranslationSpeed ?  XSpeed : MaxTranslationSpeed;
    YSpeed =  Math.abs(YSpeed) < MaxTranslationSpeed ?  YSpeed : MaxTranslationSpeed;
    RotationSpeed =  Math.abs(RotationSpeed) < MaxRotationSpeed ?  RotationSpeed : MaxRotationSpeed;

if(UpdatingTarget ==  false){
    //Apply the accelertion limiter 
    XSpeedFinal = MaxTranslationAccelerationX.calculate(XSpeed);
    YSpeedFinal = MaxTranslationAccelerationY.calculate(YSpeed);
    RotationSpeedFinal = MaxRotationAcceleration.calculate(RotationSpeed);

    // When CloseEnough == Fasle MoveToPose will end
    //This is where you will specify how close it must be for what you are doing to work
    if( (Math.abs(PoseDifferenceRotation)) < 5.0 && (Math.abs(PoseDifferenceY)) < 0.15 && (Math.abs(PoseDifferenceX)) < 0.15){
        CloseEnough = true;
    }
    else{
        CloseEnough = false;
    }
}

    if(UpdatingTarget ==  true){
        double XSpeedLine;
        double XSpeedFactor = 1 - Y_X_Ratio * Math.abs(YSpeed);

    if (0 <= XSpeedFactor) {
        XSpeedLine = XSpeedFactor*XSpeed*XCompensationValue;
    }
    else{XSpeedLine = 0;}

        XSpeedFinal = MaxTranslationAccelerationX.calculate(XSpeedLine);
        YSpeedFinal = MaxTranslationAccelerationY.calculate(YSpeed);
        RotationSpeedFinal = MaxRotationAcceleration.calculate(RotationSpeed);



    }

}


    //The following method is the lane logic this will seperate the feild into parts and define a place to drive based on where it is
    //Will have to be called in command or button to activate

    /*Writing Reminders
     * 1. Every quadrant should be constraned on all four sides or more
     * 2. Desired location should not be within the quadrent unless it is the final destination
     * 3. Robot will only drive in a staight line so make quadrants accordingly
     * 4. && = and
     * 5. & = or
     * 6. Make all quadrants and name them before starting
     * 7. Every part of the feild need a quadrent
     * 8. To make code shorter define line then quadrent based on thoose lines
     * 9. Lane to do need to change based on the allaince
   */

        //The following method will be used to drive the bot towards the drivers station
        // public static void LaneLogicIn() {
        //     //The following is the Red logic
        //     if(Constants.isRed == true){
               
        //         //Quadrant 2
        //        if(5.598 < SwerveBasePose.currentPoseX && SwerveBasePose.currentPoseX < 10.914  && 5.408 < SwerveBasePose.currentPoseY  && SwerveBasePose.currentPoseY < 8.237 ){
        //             LaneLogicPoseX = 11.914 ;
        //             LaneLogicPoseY = 7.050;
        //             LaneLogicPoseRotation = 110;
        //             System.out.println("Quadrant 2 ");
        //         }

        //         //Quadrant 3
        //         if(10.914 < SwerveBasePose.currentPoseX && SwerveBasePose.currentPoseX < 16.509  && 6.075 < SwerveBasePose.currentPoseY  && SwerveBasePose.currentPoseY < 8.237 ){
        //             LaneLogicPoseX = 15.81;
        //             LaneLogicPoseY = 6.804;
        //             LaneLogicPoseRotation = 110;
        //             System.out.println("Quadrant 3");

        //          }


        //          //Quadrant 7
        //         if(9.769 > SwerveBasePose.currentPoseX && SwerveBasePose.currentPoseX > 6.743  && 5.408 > SwerveBasePose.currentPoseY  && SwerveBasePose.currentPoseY > 2.423){
        //             LaneLogicPoseX = 12.00;
        //             LaneLogicPoseY = 4.78;
        //             LaneLogicPoseRotation = 180;
        //             System.out.println("Quadrant 7");
        //          }

        //          //Quadrant 8
        //          if(9.769 < SwerveBasePose.currentPoseX &&   -1.7140 *(SwerveBasePose.currentPoseX - 10.914) + 5.408 > SwerveBasePose.currentPoseY  &&  SwerveBasePose.currentPoseY > 0.5682 *(SwerveBasePose.currentPoseX - 15.618) + 5.046 && SwerveBasePose.currentPoseY < 5.408 && SwerveBasePose.currentPoseY > 2.423){
        //             LaneLogicPoseX = 12.40;
        //             LaneLogicPoseY = 5.24;
        //             LaneLogicPoseRotation = 180;
        //             System.out.println("Quadrant 8");

        //          }
        //          //Quadrant 9
        //          if(10.914 < SwerveBasePose.currentPoseX && SwerveBasePose.currentPoseX < 13.469  &&  -1.7140 *(SwerveBasePose.currentPoseX - 10.914) + 5.408 < SwerveBasePose.currentPoseY  && SwerveBasePose.currentPoseY < -.8853 *(SwerveBasePose.currentPoseX - 10.914) + 6.075 &&  SwerveBasePose.currentPoseY > 0.5682 *(SwerveBasePose.currentPoseX - 15.618) + 5.046){
        //         // if(10.914 < SwerveBasePose.currentPoseX && SwerveBasePose.currentPoseX < 13.469  &&  -1.7140 *(SwerveBasePose.currentPoseX - 10.914) + 5.408 < SwerveBasePose.currentPoseY  && SwerveBasePose.currentPoseY < -.8853 *(SwerveBasePose.currentPoseX - 10.914) + 6.075 &&  SwerveBasePose.currentPoseY > 0.5682 *(SwerveBasePose.currentPoseX - 15.618) + 5.046){
        //             LaneLogicPoseX = 12.60;
        //             LaneLogicPoseY = 5.75;
        //             LaneLogicPoseRotation = 180;
        //             System.out.println("Quadrant 9");
        //          }
        //          //Quadrant 10
        //         if(10.914 < SwerveBasePose.currentPoseX && SwerveBasePose.currentPoseX < 16.509  && -.8853 *(SwerveBasePose.currentPoseX - 10.914) + 6.075 < SwerveBasePose.currentPoseY  && SwerveBasePose.currentPoseY > .5682 *(SwerveBasePose.currentPoseX - 15.618) + 5.046 && SwerveBasePose.currentPoseY < 6.075){  
        //             LaneLogicPoseX = 15.129;
        //             LaneLogicPoseY = 5.536;
        //             LaneLogicPoseRotation = 180;
        //             System.out.println("Quadrant 10 ");

        //          }

        //          //Quadrant 12
        //         if(5.858 < SwerveBasePose.currentPoseX && SwerveBasePose.currentPoseX < 10.705  && 0 < SwerveBasePose.currentPoseY  && SwerveBasePose.currentPoseY < 2.423 ){
        //             LaneLogicPoseX = 12.031;
        //             LaneLogicPoseY = 2.50;
        //             LaneLogicPoseRotation = 180;
        //             System.out.println("Quadrant 12 ");

        //          }
        //          //Quadrant 13
        //         else if(10.705 < SwerveBasePose.currentPoseX && SwerveBasePose.currentPoseX < 15.50  && 0 < SwerveBasePose.currentPoseY  && SwerveBasePose.currentPoseY < .5682 *(SwerveBasePose.currentPoseX - 15.618) + 5.046 ){
        //             LaneLogicPoseX = 15.755;
        //             LaneLogicPoseY = 4.237;
        //             LaneLogicPoseRotation = 240;
        //             System.out.println("Quadrant 13"); 
        //         } 
                
        //     }
        //  }
               
        // // The following method will be used to drive the bot away from driver station
        // public static void LaneLogicOut() {
        //     //The following is the Blue logic

                
        //         // if(Constants.isRed == true){}

        //         // //Quadrant 2
        //         // if(5.598 < SwerveBasePose.currentPoseX && SwerveBasePose.currentPoseX < 10.914  && 5.408 < SwerveBasePose.currentPoseY  && SwerveBasePose.currentPoseY < 8.237 ){
        //         //     LaneLogicPoseX = 15.81;
        //         //     LaneLogicPoseY = 6.804;
        //         //     LaneLogicPoseRotation = 180;
        //         //     System.out.println("Quadrant 2 ");
        //         // }

        //         //Quadrant 3
        //          if(10.914 < SwerveBasePose.currentPoseX && SwerveBasePose.currentPoseX < 16.509  && 6.075 < SwerveBasePose.currentPoseY  && SwerveBasePose.currentPoseY < 8.237 ){
        //             LaneLogicPoseX = 9.50;
        //             LaneLogicPoseY = 6.50;
        //             LaneLogicPoseRotation = 180;
        //             System.out.println("Quadrant 3");

        //          }


        //         //  //Quadrant 7
        //         //   if(9.769 > SwerveBasePose.currentPoseX && SwerveBasePose.currentPoseX > 6.743  && 5.408 > SwerveBasePose.currentPoseY  && SwerveBasePose.currentPoseY > 2.423){
        //         //     LaneLogicPoseX = 15.129;
        //         //     LaneLogicPoseY = 5.536;
        //         //     LaneLogicPoseRotation = 0;
        //         //     System.out.println("Quadrant 7");
        //         // }
        //          //Quadrant 8
        //           if(9.769 < SwerveBasePose.currentPoseX &&   -1.7140 *(SwerveBasePose.currentPoseX - 10.914) + 5.408 > SwerveBasePose.currentPoseY  &&  SwerveBasePose.currentPoseY > 0.5682 *(SwerveBasePose.currentPoseX - 15.618) + 5.046 && SwerveBasePose.currentPoseY < 5.408 && SwerveBasePose.currentPoseY > 2.423){
        //             LaneLogicPoseX = 8.07;
        //             LaneLogicPoseY = 4.08;
        //             LaneLogicPoseRotation = 180;
        //             System.out.println("Quadrant 8");
        //          }
        //          //Quadrant 9
        //           if(10.914 < SwerveBasePose.currentPoseX && SwerveBasePose.currentPoseX < 13.469  &&  -1.7140 *(SwerveBasePose.currentPoseX - 10.914) + 5.408 < SwerveBasePose.currentPoseY  && SwerveBasePose.currentPoseY < -.8853 *(SwerveBasePose.currentPoseX - 10.914) + 6.075 &&  SwerveBasePose.currentPoseY > 0.5682 *(SwerveBasePose.currentPoseX - 15.618) + 5.046){
        //             LaneLogicPoseX = 10.04;
        //             LaneLogicPoseY = 3.63;
        //             LaneLogicPoseRotation = 180;
        //             System.out.println("Quadrant 9");
        //          }
        //          //Quadrant 10
        //           if(10.914 < SwerveBasePose.currentPoseX && SwerveBasePose.currentPoseX < 16.509  && -.8853 *(SwerveBasePose.currentPoseX - 10.914) + 6.075 < SwerveBasePose.currentPoseY  && SwerveBasePose.currentPoseY > .5682 *(SwerveBasePose.currentPoseX - 15.618) + 5.046 && SwerveBasePose.currentPoseY < 6.075){
        //             LaneLogicPoseX = 11.68; 
        //             LaneLogicPoseY = 4.41; 
        //             LaneLogicPoseRotation = 180;
        //             System.out.println("Quadrant 10 ");

        //          }

        //         //  //Quadrant 12
        //         // else if(5.858 < SwerveBasePose.currentPoseX && SwerveBasePose.currentPoseX < 10.705  && 0 < SwerveBasePose.currentPoseY  && SwerveBasePose.currentPoseY < 2.423 ){
        //         //     LaneLogicPoseX = 15.755;
        //         //     LaneLogicPoseY = 4.237;
        //         //     LaneLogicPoseRotation = 0;
        //         //     System.out.println("Quadrant 12 ");

        //         //  }
        //          //Quadrant 13
        //         if(10.705 < SwerveBasePose.currentPoseX && SwerveBasePose.currentPoseX < 15.50  && 0 < SwerveBasePose.currentPoseY  && SwerveBasePose.currentPoseY < .5682 *(SwerveBasePose.currentPoseX - 15.618) + 5.046 ){
        //             LaneLogicPoseX = 9.50;
        //             LaneLogicPoseY = 1.75;
        //             LaneLogicPoseRotation =  180;
        //             System.out.println("Quadrant 13");
        //         }
        //         // if(SwerveBasePose.currentPoseX > 0 && SwerveBasePose.currentPoseY > 0){
        //         }
             
           





public static int getQuardernt() {
 double Top_Field = 8.211; //Y Horizontal Line
 double Right_Field = 16.541; //X Verical Line
 double Left_Field = 0.0; //X Verical Line
 double Bottom_Field = 0.0; //Y Horizontal Line


 double Line_1 = -0.0565 * SwerveBasePose.currentPoseX + 5.527;
 double Line_2 = 0.0618 * SwerveBasePose.currentPoseX + 2.017;
 double Line_3 = 2.42133725; //Y Horizontal Line
 double Line_4 = 10.927; //X Verical Line
 double Line_5 = -0.404 * SwerveBasePose.currentPoseX + 6.87653752;
 double Line_6 = 2.425; // X Vertical Line
 double Line_7 = -0.06 * SwerveBasePose.currentPoseX + 6.123; // Y Intercept 
 double Line_8 = .552 * SwerveBasePose.currentPoseX + 3.616;
 double Line_9 = 14.544; // X Vertical Line
 double Line_10 = 5.799; // X Vertical Line
 double Line_11 = -.578 * SwerveBasePose.currentPoseX + 12.168; 


 if( Line_4 < SwerveBasePose.currentPoseX  && /*SwerveBasePose.currentPoseX < Right_Field &&*/
    Bottom_Field < SwerveBasePose.currentPoseY && SwerveBasePose.currentPoseY < Line_5){
    return 3;
 }

  if( Left_Field < SwerveBasePose.currentPoseX  &&
     Line_2 < SwerveBasePose.currentPoseY &&  SwerveBasePose.currentPoseY < Line_1){
    return 1;
 }

 if( Left_Field < SwerveBasePose.currentPoseX  && SwerveBasePose.currentPoseX < Line_4 &&
    Bottom_Field < SwerveBasePose.currentPoseY && (SwerveBasePose.currentPoseY < Line_1 & SwerveBasePose.currentPoseY < Line_3)&& SwerveBasePose.currentPoseY < Line_2){
    return 2;
 }
if( Left_Field < SwerveBasePose.currentPoseX  && SwerveBasePose.currentPoseX < Line_6 &&
    Line_7 < SwerveBasePose.currentPoseY && SwerveBasePose.currentPoseY < Top_Field ){
    return 4;
 }

if ( Line_6 < SwerveBasePose.currentPoseX  && SwerveBasePose.currentPoseX < Line_9 &&
    ((Line_10 < SwerveBasePose.currentPoseY && Line_7 < SwerveBasePose.currentPoseY) & (Line_11 < SwerveBasePose.currentPoseY && Line_8 < SwerveBasePose.currentPoseY)) && SwerveBasePose.currentPoseY < Top_Field){
    return 5;
 } 

 if ( Line_9 < SwerveBasePose.currentPoseX  && SwerveBasePose.currentPoseX < Right_Field && 
    Line_8 < SwerveBasePose.currentPoseY && SwerveBasePose.currentPoseY < Top_Field ){
    return 6;
}
else{
    return 0;
}
}


  static double LeadAmount = 0.1;

/*The following method take the current quadrent and move the robot to the comanded Zone */
public static void LaneLogic(boolean In, String Location) {

   // equation for a circel
    //Math.sqrt(r*r - (SwerveBasePose.currentPoseX-x)*(SwerveBasePose.currentPoseX-x))+y;
  



    double Equation_1 = 0.0565 *SwerveBasePose.currentPoseX + 5.527;
    double Equation_1_Min = 0.0;
    double Equation_1_Max = 4.673;

    double Equation_2 =  Math.sqrt(2.361 * 2.361 - (SwerveBasePose.currentPoseX-5.956)*(SwerveBasePose.currentPoseX-5.956))+3.764;
    double Equation_2_Min = 4.673;
    double Equation_2_Max = 5.847;


    double Equation_3 = 1.410; // Y Horizontal Line
    double Equation_3_Min = 4.673;
    double Equation_3_Max =  10.929 + LeadAmount;

    double Equation_4 = 6.31361033; //Y Horizontal Line
    double Equation_4_Min = 13.777;
    double Equation_4_Max =  14.539 + LeadAmount;

    double Equation_5 = Math.sqrt(2.361 * 2.361 - (SwerveBasePose.currentPoseX-5.956)*(SwerveBasePose.currentPoseX-5.956))+3.764;; //Y Horizontal Line
    double Equation_5_Min = 13.777;
    double Equation_5_Max =  13.777;
;





    if (In == true){
         double HeadingX = SwerveBasePose.currentPoseX - LeadAmount;
        if(Location == "Speacker"){
            /*Bottom Lane*/
            if(getQuardernt() == 1 ){
                    UpdatingTarget = false;
                    LaneLogicPoseX = 0.698;
                    LaneLogicPoseY = 4.352;
                    LaneLogicPoseRotation = 110;
                    }
            }
            if(getQuardernt() == 2 ){
                UpdatingTarget = true;
                LaneLogicPoseX =  HeadingX;
                if(Equation_1_Min < HeadingX && HeadingX < Equation_1_Max){      
                    LaneLogicPoseY = Equation_1;
                    LaneLogicPoseRotation = 180;}

                if(Equation_2_Min < HeadingX && HeadingX < Equation_2_Max){
                    LaneLogicPoseY = Equation_2;
                    LaneLogicPoseRotation = 180;}

                if(Equation_3_Min < HeadingX && HeadingX < Equation_3_Max){       
                    LaneLogicPoseY = Equation_3;
                    LaneLogicPoseRotation = 180;}

            }
            if(getQuardernt() == 3 ){
                    UpdatingTarget = false;
                    LaneLogicPoseX =  10.166;
                    LaneLogicPoseY = 1.410;
                    LaneLogicPoseRotation = 180;
            }

            /*Middle Lane*/

            /*Top Lane*/
 
       }
        if(Location == "Amp"){

            if(getQuardernt() == 1 ){

            }
            if(getQuardernt() == 2 ){

            }
            if(getQuardernt() == 3 ){

            }
        }
    


        if (In == false){
             double HeadingX = SwerveBasePose.currentPoseX + LeadAmount;
            

            
            if(getQuardernt() == 1 & getQuardernt() == 2 ){
                UpdatingTarget = true;
                if(Equation_1_Min < HeadingX && HeadingX < Equation_1_Max){      
                    LaneLogicPoseX =  HeadingX;
                    LaneLogicPoseY = Equation_1;
                    LaneLogicPoseRotation = 180;}

                if(Equation_2_Min < HeadingX && HeadingX < Equation_2_Max){
                    LaneLogicPoseX =  HeadingX;
                    LaneLogicPoseY = Equation_2;
                    LaneLogicPoseRotation = 180;}

                if(Equation_3_Min < HeadingX && HeadingX < Equation_3_Max){       
                    LaneLogicPoseX =  HeadingX;
                    LaneLogicPoseY = Equation_3;
                    LaneLogicPoseRotation = 180;}

            }
            if(getQuardernt() == 3 ){
                    UpdatingTarget = false;
                    LaneLogicPoseX =  12.27;
                    LaneLogicPoseY = 1.292;
                    LaneLogicPoseRotation = 150;
            }
    }
}



   @Override
  public void periodic() {
    //Runs DogLog to log values to USB
    DogLog();

    // System.out.println("Slew" + MaxTranslationAcceleration.calculate(XSpeed)); 
    //System.out.println("Quadrent is "+ getQuardernt()); 
    }

    private void DogLog(){
        // //Main Values
        // DogLog.log("/SelfDriving/Quardent", getQuardernt());
        // DogLog.log("/SelfDriving/CloseEnough", CloseEnough);

        // //Published Pose Values
        // DogLog.log("/SelfDriving/Position/TargetX", TargetPoseX);
        // DogLog.log("/SelfDriving/Position/TargetY", TargetPoseY);
        // DogLog.log("/SelfDriving/Position/TargetRotation", TargetPoseRotation);

        // //Lane Logic Values
        // DogLog.log("/SelfDriving/LaneLogicValue/Heading", LaneLogicPoseX);
        // DogLog.log("/SelfDriving/LaneLogicValue/Odometry", LaneLogicPoseY);
        // DogLog.log("/SelfDriving/LaneLogicValue/Odometry", LaneLogicPoseRotation);
        // DogLog.log("/SelfDriving/LaneLogicValue/Odometry", LaneLogicFinalDestination);
        // DogLog.log("/SelfDriving/LaneLogicValue/Odometry", UpdatingTarget);

        // //Pose Difference vlaues
        // DogLog.log("/SelfDriving/Difference/DifferenceX", PoseDifferenceX );
        // DogLog.log("/SelfDriving/Difference/DifferenceY", PoseDifferenceY);
        // DogLog.log("/SelfDriving/Difference/DifferenceY", PoseDifferenceRotation);

        // //Speeds in each derection if note ditection XSpeed is forward
        // DogLog.log("/SelfDriving/Speeds/XSpeed", XSpeedFinal);
        // DogLog.log("/SelfDriving/Speeds/YSpeed", YSpeedFinal);
        // DogLog.log("/SelfDriving/Speeds/RotationSpeed", RotationSpeedFinal);

        // // Value for auto setup
        // // DogLog.log("/SelfDriving/AutoSetUp/MoveLeft", AutoSetUpMoveLeft);
        // // DogLog.log("/SelfDriving/AutoSetUp/MoveRight", AutoSetUpMoveRight);
        // // DogLog.log("/SelfDriving/AutoSetUp/MoveForward", AutoSetUpMoveForward);
        // // DogLog.log("/SelfDriving/AutoSetUp/MoveBackward", AutoSetUpMoveBackward);
        // // DogLog.log("/SelfDriving/AutoSetUp/RotateCouterCloskWise", AutoSetUpRotateCounterClockWise);
        // // DogLog.log("/SelfDriving/AutoSetUp/RotateCloskWise", AutoSetUpRotateCloskWise);
        // // DogLog.log("/SelfDriving/AutoSetUp/CloseEnought", AutoSetUpCloseEnough);

    }
}