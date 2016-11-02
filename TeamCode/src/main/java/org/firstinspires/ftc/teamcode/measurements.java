package org.firstinspires.ftc.teamcode;

/**
 * Created by mail2 on 10/31/2016.
 */

/***
 * includes a bunch of important measurements
 */
public class measurements {
    public float mmPerInch = (float) 25.4; //please don't ask why I am using millimeters instead of inches
    public float perimeterLength = ((12 * 12) - 2) * this.mmPerInch;
    public float tileLength = perimeterLength / 12;
    public float wheelDiameter = 4 * this.mmPerInch;
    public float widthOfDrive = 15 * this.mmPerInch;
    public float heightOfDrive = 14 * this.mmPerInch;
    public float robotWidth = (float) (17.5 * this.mmPerInch);
    public float robotDepth = (float) (17.5 * this.mmPerInch);
    public float wheelDiagonal = (float) (2 * Math.sqrt((this.heightOfDrive * this.heightOfDrive) + (this.widthOfDrive * this.widthOfDrive)));
    public int ppr = 280;
    public int ppd = ppr * 360;
    public float pi = (float) 3.1415926535897932384626433832795;
}
