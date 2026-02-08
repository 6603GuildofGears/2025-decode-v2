// package org.firstinspires.ftc.teamcode.pedroPathing.Pipelines;

// import org.firstinspires.ftc.teamcode.pedroPathing.Pipelines.Motor_PipeLine;
// import org.firstinspires.ftc.teamcode.pedroPathing.Pipelines.LimelightPipeline;
// import com.qualcomm.robotcore.hardware.HardwareMap; 


// public class TurretPipeline {

   
//     private double KP = 0.0001;

//     private double KD = 0.0;

//     private double GoalX = 0;

//     private double lastError = 0;

//     private double errorTolorance = 0.2;

//     private final double MAX_POWER = 0.8;

//     private double power = 0;

//     private final ElapsedTime timer = new ElapsedTime();

//     public void int(HardwareMap HwMap){

//             Motor_PipeLine motors = new Motor_PipeLine(HwMap); 
//             LimelightPipeline limelight = new LimelightPipeline(HwMap);

//             turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

//     }

//     public void setKP (Double newKP){
//         KP = newKP;
//     }

//     public double getKP(){
//         return KP;
//     }

//      public void setKD (Double newKD){
//         KD = newKD;
//     }

//     public double getKD(){
//         return KD;
//     }
    
//     public void resetTimer(){
//         timer.reset();
//     }   

//     public void update(limelightResult result){
//         double deltaTime = timer.seconds();
//         timer.reset();

//         if(result == null){
//             power = 0;
//             lastError = 0;  
//             return;
//         }

//         double error = GoalX - result.x;

//         double pTerm = KP * error;

//         double dTerm = 0;
//         if(deltaTime > 0){
//             dTerm = KD * ((error - lastError) / deltaTime *KD);
//         }



//         if (Math.abs(error) < errorTolorance){
//             power = 0;
//         } else {
//             power = Range.clip(pTerm + dTerm, -MAX_POWER, MAX_POWER);
//         }

//         turret.setPower(power);

//         lastError = error;
//     }
    
// }