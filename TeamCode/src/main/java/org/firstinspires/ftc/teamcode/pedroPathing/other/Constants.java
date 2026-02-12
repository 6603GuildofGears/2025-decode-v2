package org.firstinspires.ftc.teamcode.pedroPathing.Autos.other;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
    .mass(12.73)
    .forwardZeroPowerAcceleration( -49.18626972944772)
    .lateralZeroPowerAcceleration(  -94.09409090995455)
    .translationalPIDFCoefficients(new PIDFCoefficients(0.075, 0, 0.001875, 0.0475))
    .headingPIDFCoefficients(new PIDFCoefficients(0.5, 0, 0.0775, 0.005))
    .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.02, 0, 0.001, 0.6, 0))
    .centripetalScaling(0.00035)
    ;
    public static MecanumConstants driveConstants = new MecanumConstants()
        .maxPower(1)
        .rightFrontMotorName("backRight")
        .rightRearMotorName("frontRight")
        .leftRearMotorName("backLeft")
        .leftFrontMotorName("frontLeft")
        .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
        .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
        .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
        .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)
        .xVelocity(78.51130640225148)
        .yVelocity(59.03510242372048)
        ;


        public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(7.324556)  
            .strafePodX(0)       
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            ;




    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

   public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
}
}