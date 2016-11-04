package org.firstinspires.ftc.teamcode;

/**
 * Created by mail2 on 10/31/2016.
 */

/***
 * includes a bunch of important measurements
 */
public class measurements {
    public double mmPerInch = 25.4; //please don't ask why I am using millimeters instead of inches
    public double perimeterLength = ((12 * 12) - 2) * this.mmPerInch;
    public double tileLength = perimeterLength / 12;
    public double wheelDiameter = 4 * this.mmPerInch;
    public double widthOfDrive = (13 + 3 / 8) * this.mmPerInch;
    public double heightOfDrive = (14 + 3 / 8) * this.mmPerInch;
    public double robotWidth = (17.5 * this.mmPerInch);
    public double robotDepth = (17.5 * this.mmPerInch);
    public double wheelDiagonal = (2 * Math.sqrt((this.heightOfDrive * this.heightOfDrive) + (this.widthOfDrive * this.widthOfDrive)));
    public int ppr = 280 * 4;
    public double pi = 3.1415926535897932384626433832795;
}
