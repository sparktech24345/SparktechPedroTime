package Experimental.StatesAndPositions;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;

public class AutoOfSpecStatesAndPos {

    
    // =-=-=-=-=-=-=-= REQUIRED CONSTANTS =-=-=-=-=-=-=-= \\

    public static final float scoringBarX = -2f;
    public static final float scoringBarY = 40.6f;

    public static final float wallPickUpX = -42f;
    public static final float wallPickUpY = 71f;
    public static final float wallAdderY = 2f;


    /* * * * * * * * * * * * * * * * * * * * * * * * *
    *               ALL POSES NEEDED                 *
    * * * * * * * * * * * * * * * * * * * * * * * * */
    
    
    public static final Pose startPose = new Pose(-10, 70, Math.toRadians(90));


    // =-=-=-=-=-=-=-= SCORING BAR POSES =-=-=-=-=-=-=-= \\

    
    public static final Pose scoringBarPreloadSpecPose = new Pose(scoringBarX, scoringBarY, Math.toRadians(90));
    public static final Pose scoringBarFirstSpecPose = new Pose(scoringBarX-2, scoringBarY+1, Math.toRadians(90));
    public static final Pose scoringBarSecondSpecPose = new Pose(scoringBarX-3, scoringBarY+1, Math.toRadians(90));
    public static final Pose scoringBarThirdSpecPose = new Pose(scoringBarX-4, scoringBarY+1, Math.toRadians(90));
    public static final Pose scoringBarFourthSpecPose = new Pose(scoringBarX-4, scoringBarY+1, Math.toRadians(90));
    
    
    // =-=-=-=-=-=-=-= SPECIMEN WALL PICKUP POSES =-=-=-=-=-=-=-= \\
    
    
    public static final Pose firstSpecimenPickUpPose = new Pose(wallPickUpX, wallPickUpY, Math.toRadians(90)); 
    public static final Pose secondSpecimenPickUpPose = new Pose(wallPickUpX, wallPickUpY+wallAdderY, Math.toRadians(90)); 
    public static final Pose thirdSpecimenPickUpPose = new Pose(wallPickUpX, wallPickUpY+wallAdderY+0.2, Math.toRadians(90)); 
    public static final Pose fourthSpecimenPickUpPose = new Pose(wallPickUpX, wallPickUpY+wallAdderY+0.4, Math.toRadians(90));


    // =-=-=-=-=-=-=-= GROUND SAMPLES PICKUP POSES =-=-=-=-=-=-=-= \\


    public static final Pose firstSamplePickUpPos = new Pose(-58.5 - 2,60,Math.toRadians(112)); 
    public static final Pose secondSamplePickUpPos = new Pose(-59 - 2, 61, Math.toRadians(100)); 
    public static final Pose thirdSamplePickUpPos = new Pose(-51 - 2, 64, Math.toRadians(64)); 

    // PARKING POSE
    
    public static final Pose parkingPose = new Pose(-55,70 - 0.5,Math.toRadians(90)); //parking
    
    public static Path 
            startPath = new Path(new BezierLine(new Point(startPose), new Point(scoringBarPreloadSpecPose))),

            pickUpFirstSpec,
            scoreFirstSpec,

            pickUpSecondSpec,
            scoreSecondSpec,

            pickUpThirdSpec,
            scoreThirdSpec,

            pickUpFourthSpec,
            scoreFourthSpec,

            pickUpFifthSpec,
            scoreFifthSpec,
    
            parking;

    public static PathChain 
            goToPickUpFirstSample,
            goToPickUpSecondSample,
            goToPickUpThirdSample;
    
    public static void InitAllPaths(Follower  follower_instance) {
        startPath = new Path(new BezierLine(new Point(startPose), new Point(scoringBarPreloadSpecPose)));
        startPath.setLinearHeadingInterpolation(startPose.getHeading(), scoringBarPreloadSpecPose.getHeading());
        
        // 3 2 1 logic
        goToPickUpFirstSample = follower_instance.pathBuilder()
                .addPath(new BezierLine(new Point(scoringBarPreloadSpecPose), new Point(thirdSamplePickUpPos)))
                .setLinearHeadingInterpolation(scoringBarPreloadSpecPose.getHeading(), thirdSamplePickUpPos.getHeading())
                .build();

        goToPickUpSecondSample= follower_instance.pathBuilder()
                .addPath(new BezierLine(new Point(thirdSamplePickUpPos), new Point(secondSamplePickUpPos)))
                .setLinearHeadingInterpolation(thirdSamplePickUpPos.getHeading(), secondSamplePickUpPos.getHeading())
                .build();

        goToPickUpThirdSample= follower_instance.pathBuilder()
                .addPath(new BezierLine(new Point(secondSamplePickUpPos), new Point(firstSamplePickUpPos)))
                .setLinearHeadingInterpolation(secondSamplePickUpPos.getHeading(), firstSamplePickUpPos.getHeading())
                .build();

        //first spec //changed logic to for 3 2 1 logic
        pickUpFirstSpec = new Path(new BezierLine(new Point(firstSamplePickUpPos), new Point(firstSpecimenPickUpPose)));
        pickUpFirstSpec.setLinearHeadingInterpolation(firstSamplePickUpPos.getHeading(), firstSpecimenPickUpPose.getHeading());

        scoreFirstSpec = new Path(new BezierLine(new Point(firstSpecimenPickUpPose), new Point(scoringBarFirstSpecPose)));
        scoreFirstSpec.setLinearHeadingInterpolation(firstSpecimenPickUpPose.getHeading(), scoringBarFirstSpecPose.getHeading());
        //second spec

        pickUpSecondSpec = new Path(new BezierLine(new Point(scoringBarFirstSpecPose), new Point(secondSpecimenPickUpPose)));
        pickUpSecondSpec.setLinearHeadingInterpolation(scoringBarFirstSpecPose.getHeading(), secondSpecimenPickUpPose.getHeading());

        scoreSecondSpec = new Path(new BezierLine(new Point(secondSpecimenPickUpPose), new Point(scoringBarSecondSpecPose)));
        scoreSecondSpec.setLinearHeadingInterpolation(secondSpecimenPickUpPose.getHeading(), scoringBarSecondSpecPose.getHeading());



        pickUpThirdSpec = new Path(new BezierLine(new Point(scoringBarSecondSpecPose), new Point(thirdSpecimenPickUpPose)));
        pickUpThirdSpec.setLinearHeadingInterpolation(scoringBarSecondSpecPose.getHeading(), thirdSpecimenPickUpPose.getHeading());

        scoreThirdSpec = new Path(new BezierLine(new Point(thirdSpecimenPickUpPose), new Point(scoringBarThirdSpecPose)));
        scoreThirdSpec.setLinearHeadingInterpolation(thirdSpecimenPickUpPose.getHeading(), scoringBarThirdSpecPose.getHeading());


        //fourth spec

        pickUpFourthSpec = new Path(new BezierLine(new Point(scoringBarThirdSpecPose), new Point(fourthSpecimenPickUpPose)));
        pickUpFourthSpec.setLinearHeadingInterpolation(scoringBarThirdSpecPose.getHeading(), fourthSpecimenPickUpPose.getHeading());

        scoreFourthSpec = new Path(new BezierLine(new Point(fourthSpecimenPickUpPose), new Point(scoringBarFourthSpecPose)));
        scoreFourthSpec.setLinearHeadingInterpolation(fourthSpecimenPickUpPose.getHeading(), scoringBarFourthSpecPose.getHeading());


        //parking
        parking = new Path(new BezierLine(new Point(scoringBarFourthSpecPose), new Point(parkingPose)));
        parking.setLinearHeadingInterpolation(scoringBarFourthSpecPose.getHeading(), parkingPose.getHeading());
    }
}
