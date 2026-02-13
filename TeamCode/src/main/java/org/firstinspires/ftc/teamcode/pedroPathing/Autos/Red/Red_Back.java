package org.firstinspires.ftc.teamcode.pedroPathing.Autos.Red;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.util.Timer;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.*;
import com.pedropathing.paths.*;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.other.Constants;

@Autonomous(name = "PEDRO - Red Back Auto", group = "Red")
public class Red_Back extends OpMode {
    private Follower follower;
    private Timer pathTimer, opmodeTimer;


        public enum PathState {
            DRIVE_STARTPOSE_TO_SHOOTPOSE,
            SHOOT_PRELOAD,
            DRIVE_SHOOT_PRELOAD_TO_INTAKEPOSE,
            DRIVE_INTAKEPOSE_TO_INTAKE1,
            DRIVE_INTAKE1_TO_SHOOTPOSE,
            SHOOT_INTAKE1,
            DRIVE_SHOOTPOSE_TO_INTAKEPOSE2,
            DRIVE_INTAKEPOSE2_TO_INTAKE2,
            DRIVE_INTAKE2_TO_SHOOTPOSE,
            SHOOT_INTAKE2,
            DRIVE_SHOOTPOSE_TO_ENDPOSE,


        }

              PathState pathState;


    private final Pose startPose = new Pose(84, 9, Math.toRadians(0));

    private final Pose shootPose = new Pose(84, 20, Math.toRadians(0));    // Shooting position

    private final Pose intakePose = new Pose(103, 36, Math.toRadians(0));    // Intake position

    private final Pose intake1 = new Pose(106, 36, Math.toRadians(0));

    private final Pose intakePose2 = new Pose(135,  9.5, Math.toRadians(0));

    private final Pose intake2 = new Pose(135, 9.5, Math.toRadians(0));

    private final Pose endPose = new Pose(120, 36, Math.toRadians(0));  // End position
   // Shooter hardware
   
    private ElapsedTime shooterTimer = new ElapsedTime();
    private boolean shooterStarted = false;
    private boolean pathStarted = false;


    private PathChain driveStartPoseShootPose;
    private PathChain driveShootPreloadToIntakePose;
    private PathChain driveIntakePoseToIntake1;
    private PathChain driveIntake1ToShootPose;
    private PathChain driveShootPoseToIntakePose2;
    private PathChain driveIntakePose2ToIntake2;
    private PathChain driveIntake2ToShootPose;
    private PathChain driveShootPoseToEndPose;


 public void buildPaths() {
     
     driveStartPoseShootPose = follower.pathBuilder()
            .addPath(new BezierLine(startPose, shootPose))
            .setConstantHeadingInterpolation(shootPose.getHeading())
            .build();
         driveShootPreloadToIntakePose = follower.pathBuilder()
            .addPath(new BezierLine(shootPose, intakePose))
            .setConstantHeadingInterpolation(intakePose.getHeading())
            .build();
        driveIntakePoseToIntake1 = follower.pathBuilder()
            .addPath(new BezierLine(intakePose, intake1))
            .setConstantHeadingInterpolation(intake1.getHeading())
            .build();
        driveIntake1ToShootPose = follower.pathBuilder()
            .addPath(new BezierLine(intake1, shootPose))
            .setLinearHeadingInterpolation(intake1.getHeading(), shootPose.getHeading())
            .build();
        driveShootPoseToIntakePose2 = follower.pathBuilder()
            .addPath(new BezierLine(shootPose, intakePose2))
            .setConstantHeadingInterpolation(intakePose2.getHeading())
            .build();   
        driveIntakePose2ToIntake2 = follower.pathBuilder()
            .addPath(new BezierLine(intakePose2, intake2))
            .setConstantHeadingInterpolation(intake2.getHeading())
            .build();
        driveIntake2ToShootPose = follower.pathBuilder()
            .addPath(new BezierLine(intake2, shootPose))
            .setLinearHeadingInterpolation(intake2.getHeading(), shootPose.getHeading())
            .build();
        driveShootPoseToEndPose = follower.pathBuilder()
            .addPath(new BezierLine(shootPose, endPose))
            .setConstantHeadingInterpolation(endPose.getHeading())
            .build();
    }

     public void statePathUpdate() {
        switch (pathState) {
            case DRIVE_STARTPOSE_TO_SHOOTPOSE:
                if (!pathStarted) {
                    follower.followPath(driveStartPoseShootPose, true);
                    pathStarted = true;
                }
                if (follower.isBusy()) {
                    follower.update();
                } else {
                    pathState = PathState.SHOOT_PRELOAD;
                    pathStarted = false;
                }

                break;
                
            case SHOOT_PRELOAD:

              if(!follower.isBusy()){
                    // Wait 2 seconds then open blocker and run intake
                    if (shooterTimer.seconds() >= 2.0) {
                        
                    }
                    
                    // After 6 seconds total, move to next state
                    if (shooterTimer.seconds() >= 7) {
                        pathState = PathState.DRIVE_SHOOT_PRELOAD_TO_INTAKEPOSE;
                        shooterStarted = false;
                    }
                 
                    telemetry.addLine("Path 1 Done");
                }
                break;


               case DRIVE_SHOOT_PRELOAD_TO_INTAKEPOSE:
                if (!pathStarted) {
                    follower.followPath(driveShootPreloadToIntakePose, true);
                    
                    // Turn off shooter and close blocker when moving to intake
                    
                    pathStarted = true;
                }
                
                if (pathStarted && !follower.isBusy()) {
                    pathState = PathState.DRIVE_INTAKEPOSE_TO_INTAKE1;
                    pathStarted = false;
                }
                break;

                case DRIVE_INTAKEPOSE_TO_INTAKE1:
                if (!pathStarted) {
                    follower.followPath(driveIntakePoseToIntake1, true);
                    
                    // Turn on intake during path
                   
                    pathStarted = true;
                }
                
                if (pathStarted && !follower.isBusy()) {
                    pathState = PathState.DRIVE_INTAKE1_TO_SHOOTPOSE;
                    pathStarted = false;
                }
                break;

                case DRIVE_INTAKE1_TO_SHOOTPOSE:
                if (!pathStarted) {
                    follower.followPath(driveIntake1ToShootPose, true);
                    pathStarted = true;
                }
                
                // Start shooter when path is halfway done
                if (follower.getCurrentTValue() >= 0.5 && !shooterStarted) {
                  
                    shooterTimer.reset();
                    shooterStarted = true;
                }
                
                if (!follower.isBusy()) {
                    pathState = PathState.SHOOT_INTAKE1;
                    pathStarted = false;
                }
                break;


                case SHOOT_INTAKE1:
                if (!follower.isBusy()) {
                    // Wait 2 seconds then open blocker and run intake
                    if (shooterTimer.seconds() >= 2.0) {
                        
                    }
                    
                    // After 6 seconds total, move to next state
                    if (shooterTimer.seconds() >= 8) {
                        pathState = PathState.DRIVE_SHOOTPOSE_TO_INTAKEPOSE2;
                        shooterStarted = false;

                    }
                    
                    telemetry.addLine("Sample 1 Shot");
                }
                break;

                   case DRIVE_SHOOTPOSE_TO_INTAKEPOSE2:
                if (!pathStarted) {
                    follower.followPath(driveShootPoseToIntakePose2, true);
                    // Turn off shooter and close blocker when moving to intake
                 
                    pathStarted = true;
                }
                
                if (pathStarted && !follower.isBusy()) {
                    pathState = PathState.DRIVE_INTAKEPOSE2_TO_INTAKE2;
                    pathStarted = false;
                }
                break;  

                case DRIVE_INTAKEPOSE2_TO_INTAKE2:
                if (!pathStarted) {
                    follower.followPath(driveIntakePose2ToIntake2, true);
                    pathStarted = true;
                }
                
                if (pathStarted && !follower.isBusy()) {
                    pathState = PathState.DRIVE_INTAKE2_TO_SHOOTPOSE;
                    pathStarted = false;
                }
                break;

                 case DRIVE_INTAKE2_TO_SHOOTPOSE:
                if (!pathStarted) {
                    follower.followPath(driveIntake2ToShootPose, true);
                    pathStarted = true;
                }
                
                // Start shooter when path is halfway done
                if (follower.getCurrentTValue() >= 0.5 && !shooterStarted) {
                    shooterTimer.reset();
                    shooterStarted = true;
                }
                
                if (!follower.isBusy()) {
                    pathState = PathState.SHOOT_INTAKE2;
                    pathStarted = false;
                }
                break;

             case SHOOT_INTAKE2:
                if (!follower.isBusy()) {
                    // Wait 2 seconds then open blocker and run intake
                    if (shooterTimer.seconds() >= 2.0) {
                     
                    }
                    
                    telemetry.addLine("Sample 2 Shot");
                    
                    // After 6 seconds total, move to end position
                    if (shooterTimer.seconds() >= 6) {
                        // Turn off shooter and intake, close blocker
                       
                        
                        pathState = PathState.DRIVE_SHOOTPOSE_TO_ENDPOSE;
                        shooterStarted = false;
                    }
                }
                break;

                case DRIVE_SHOOTPOSE_TO_ENDPOSE:
                if (!pathStarted) {
                    follower.followPath(driveShootPoseToEndPose, true);
                    pathStarted = true;
                }
                
                if (pathStarted && !follower.isBusy()) {
                    telemetry.addLine("Auto Complete");
                    pathStarted = false;
                    requestOpModeStop();
                }
                break;


            default:
                telemetry.addLine("No valid path state");
                break;   

        }


     }

         public void statePathUpdate( PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }



             @Override
    public void init() {
    
    pathState = PathState.DRIVE_STARTPOSE_TO_SHOOTPOSE;
    pathTimer = new Timer();
    opmodeTimer = new Timer();
    opmodeTimer.resetTimer();
    follower = Constants.createFollower(hardwareMap);
    
    // Initialize shooter hardware
   
    buildPaths();
    follower.setPose(startPose);

    }

     public void start () {
    opmodeTimer.resetTimer();
    pathState = PathState.DRIVE_STARTPOSE_TO_SHOOTPOSE;

    }

    @Override
    public void loop() {

        follower.update();
        
        statePathUpdate();

        telemetry.addData("Path state", pathState.toString());
        telemetry.addData("X", follower.getPose().getX());       
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Path time (s): ", pathTimer.getElapsedTimeSeconds());
        telemetry.update();
    
    }



 } 
    