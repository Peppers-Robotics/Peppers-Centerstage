package org.firstinspires.ftc.teamcode.Auto;

import android.util.Pair;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Mat;

import java.util.Vector;

public class PurePursuit {
    private double Radius;
    private Localizer localizer;
    private Vector<Pose2d> checkPoints;
    private Pose2d lastHeadingPose;
    private boolean isOnLastSegment = false;
    public PurePursuit(Localizer localizer, Vector<Pose2d> checkPoints, double radius){
        this.Radius = radius;
        this.localizer = localizer;
        this.checkPoints = checkPoints;
        lastHeadingPose = checkPoints.get(0);
        TelemetryPacket pack = new TelemetryPacket();
        for(int i = 0; i < checkPoints.size() - 1; i++){
            pack.fieldOverlay()
                    .setFill("black")
                    .strokeLine(checkPoints.elementAt(i).getX(), checkPoints.elementAt(i).getY(), checkPoints.elementAt(i+1).getX(), checkPoints.elementAt(i+1).getY());
            ;
            FtcDashboard.getInstance().sendTelemetryPacket(pack);
        }

    }
    private Pose2d getFirstDegreeCoeficients(Pose2d p1, Pose2d p2){
        if(p1.getY() == p2.getY()) return new Pose2d();
        if(p1.getX() == p2.getX()) return new Pose2d();
        double a = (p1.getY() - p2.getY()) / (p1.getX() - p2.getX());
        double b = p1.getY() - a * p1.getX();

        return new Pose2d(a, b);
    }

    public static double Distance(Pose2d A, Pose2d B){
        return Math.sqrt((A.getX() - B.getX()) * (A.getX() - B.getX()) + (A.getY() - B.getY()) * (A.getY() - B.getY()));
    }
    public Pose2d getNextHeadingPoint(){

        if(Distance(lastHeadingPose, checkPoints.lastElement()) <= Radius && isOnLastSegment) return checkPoints.lastElement();

        for(int i = 0; i < checkPoints.size() - 1; i++){
            double a = getFirstDegreeCoeficients(checkPoints.elementAt(i), checkPoints.elementAt(i + 1)).getX();
            double b = getFirstDegreeCoeficients(checkPoints.elementAt(i), checkPoints.elementAt(i + 1)).getY();
            Pose2d thisPos = localizer.getPoseEstimate();
            Pose2d n1 = new Pose2d();
            Pose2d n2 = new Pose2d();

            double alpha, beta, gamma, delta;

            if(checkPoints.elementAt(i).getX() == checkPoints.elementAt(i+1).getX()){
                alpha = 1;
                beta = -2 * thisPos.getY();
                double x = checkPoints.elementAt(i).getX();
                gamma = x * x - 2 * x * thisPos.getX() + thisPos.getX() * thisPos.getX() + thisPos.getY() * thisPos.getY() - Radius * Radius;

                delta = (beta * beta - 4 * alpha * gamma);
                if(delta < 0) continue;


                n1 = new Pose2d(x, (-beta + Math.sqrt(delta)) / (2 * alpha));
                n2 = new Pose2d(x, (-beta - Math.sqrt(delta)) / (2 * alpha));

            } else if(checkPoints.elementAt(i).getY() == checkPoints.elementAt(i+1).getY()){
                alpha = 1;
                beta = -2 * thisPos.getX();
                double y = checkPoints.elementAt(i).getY();
                gamma = y * y - 2 * y * thisPos.getY() + thisPos.getY() * thisPos.getY() - Radius * Radius;

                delta = (beta * beta - 4 * alpha * gamma);
                if(delta < 0) continue;

                n1 = new Pose2d((-beta + Math.sqrt(delta)) / (2 * alpha), checkPoints.elementAt(i).getY());
                n2 = new Pose2d((-beta - Math.sqrt(delta)) / (2 * alpha), checkPoints.elementAt(i).getY());
            } else {
                alpha = 1 + a * a;
                beta = -2 * thisPos.getX() + 2 * a * (b - thisPos.getY());
                gamma = thisPos.getX() * thisPos.getX() + b * b - 2 * b * thisPos.getY() + thisPos.getY() * thisPos.getY() - Radius * Radius;

                delta = (beta * beta - 4 * alpha * gamma);

                if(delta < 0) continue;

                double x = (-beta + Math.sqrt(delta)) / (2 * alpha);
                double y = a * x + b;
                n1 = new Pose2d(x, y);
                x = (-beta - Math.sqrt(delta)) / (2 * alpha);
                y = a * x + b;
                n2 = new Pose2d(x, y);
            }
            if(i == checkPoints.size() - 2) isOnLastSegment = true;

            if(Distance(n1, checkPoints.elementAt(i + 1)) < Distance(n2, checkPoints.elementAt(i + 1))){
                double head = -Math.atan(getFirstDegreeCoeficients(lastHeadingPose, n1).getX());
                lastHeadingPose = new Pose2d(n1.getX(), n1.getY(), head);
            } else {
                double head = -Math.atan(getFirstDegreeCoeficients(lastHeadingPose, n2).getX());
                lastHeadingPose = new Pose2d(n2.getX(), n2.getY(), head);
            }
        }

        return lastHeadingPose;
    }
}
