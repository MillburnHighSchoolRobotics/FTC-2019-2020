package org.firstinspires.ftc.teamcode.autonomous;

import java.util.*;

/**
 * Created by Sahil Shah on 10/12/2018.
 * Our cubic spline system.
 */
public class Spline {
    private static double[][] x_interpolant;
    private static double[][] y_interpolant;
    private static double[][] domain;
    private static double[][] interpolant;

    public static void spline(double[] x, double[] y, double[] time) {
        setDomain(time);
        x_interpolant = interpolate(time, x);
        y_interpolant = interpolate(time, y);
    }
    public static double[][] interpolate(double[] xArr, double[] yArr) {
        LinkedList<double[]> path = new LinkedList<>();
        for (int a = 0; a < xArr.length; a++) {
            path.add(new double[] {xArr[a],yArr[a]});
        }

        //setup
        double[][] matrix = new double[(path.size()-1)*4][(path.size()-1)*4];
        for (int x = 0; x < matrix.length; x++) {
            for (int y = 0; y < matrix[x].length; y++) {
                matrix[x][y] = 0;
            }
        }
        double[] matrixY = new double[matrix.length];
        for (int x = 0; x < matrix.length; x++) {
            matrixY[x] = 0;
        }
        double[][] ranges = new double[path.size()-1][2];
        int row = 0;


        //points
        for (int a = 0; a < path.size()-1; a++) {
            for (int b = 0; b < 2; b++,row++) {
                for (int c = 0; c < 4; c++) {
                    matrix[row][a*4+c] = Math.pow(path.get(a+b)[0],3-c);
                }
                matrixY[row] = path.get(a+b)[1];
                ranges[a][b] = path.get(a+b)[0];
            }
        }


        //first derivative
        for (int a = 0; a < path.size()-2; a++, row++) {
            for (int b = 0; b < 3; b++) {
                matrix[row][a*4+b] = (3-b) * Math.pow(path.get(a+1)[0],2-b);
            }
            for (int b = 4; b < 7; b++) {
                matrix[row][a*4+b] = (b-7) * Math.pow(path.get(a+1)[0],6-b);
            }
        }


        //second derivative
        for (int a = 0; a < path.size()-2; a++, row++) {
            for (int b = 0; b < 2; b++) {
                matrix[row][a*4+b] = (2-b) * (3-b) * Math.pow(path.get(a+1)[0],1-b);
            }
            for (int b = 4; b < 6; b++) {
                matrix[row][a*4+b] = (b-6) * (7-b) * Math.pow(path.get(a+1)[0],5-b);
            }
        }


        //boundary
        matrix[row][0] = 6*path.get(0)[0];
        matrix[row][1] = 2;
        row++;
        matrix[row][matrix.length-4] = 6*path.get(path.size()-1)[0];
        matrix[row][matrix.length-3] = 2;


        //Gaussian Elimination
        double[][] coefficients = GaussianElimination(matrix, matrixY, path);
        return coefficients;
    }
    public static double[][] GaussianElimination(double[][] matrix, double[] matrixY, LinkedList<double[]> path) {
        //Row Echelon Form
        for (int x = 0; x < matrixY.length; x++) {
            int f = x;
            for (int y = x + 1; y < matrixY.length; y++) {
                //check for pivot
                if (Math.abs(matrix[y][x]) > Math.abs(matrix[f][x])) {
                    f = y;
                }
            }

            //switch in matrix and matrixY
            double[] matrixTemp = matrix[x];
            matrix[x] = matrix[f];
            matrix[f] = matrixTemp;
            double matrixYTemp = matrixY[x];
            matrixY[x] = matrixY[f];
            matrixY[f] = matrixYTemp;

            //find the x-th pivot
            for (int y = x + 1; y < matrixY.length; y++) {
                double alpha = matrix[y][x]/matrix[x][x];
                matrixY[y] -= alpha * matrixY[x];
                for (int z = x; z < matrixY.length; z++) {
                    matrix[y][z] -= alpha * matrix[x][z];
                }
            }
        }

        //Find coefficients a,b,c,d for each polynomial piece of the spline
        double[] coeffList = new double[matrixY.length];
        for (int a = matrixY.length-1; a >= 0; a--) {
            double beta = 0.0;
            for (int b = a+1; b < matrixY.length; b++) {
                beta += matrix[a][b]*coeffList[b];
            }
            double difference = matrixY[a] - beta;
            coeffList[a] = difference/matrix[a][a];
        }
        double[][] coeff = new double[path.size()-1][4];
        for (int m = 0; m < path.size()-1; m++) {
            for (int n = 0; n < coeff[m].length; n++) {
                coeff[m][n] = Math.round(coeffList[(4*m)+n]*1000.0)/1000.0;
            }
        }
        return coeff;
    }
    public static void setDomain(double[] time) {
        domain = new double[time.length-1][2];
        for (int x = 0; x < time.length-1; x++) {
            domain[x][0] = time[x];
            domain[x][1] = time[x+1];
        }
    }
    public static double[] getCoordinate(double time) {
        int d = -1;
        for (int t = 0; t < domain.length; t++) {
            if (time >= domain[t][0] && time <= domain[t][1]) {
                d = t;
            }
        }
        if (d == -1) {
            throw new ArrayIndexOutOfBoundsException("Time is not in bounds");
        }
        double x = (x_interpolant[d][0]*Math.pow(time,3))+(x_interpolant[d][1]*Math.pow(time,2))+(x_interpolant[d][2]*time)+(x_interpolant[d][3]);
        double y = (y_interpolant[d][0]*Math.pow(time,3))+(y_interpolant[d][1]*Math.pow(time,2))+(y_interpolant[d][2]*time)+(y_interpolant[d][3]);
        double[] coordinate = {x,y};
        return coordinate;
    }
    public static double getAngle(double time) {
        int d = -1;
        for (int t = 0; t < domain.length; t++) {
            if (time >= domain[t][0] && time <= domain[t][1]) {
                d = t;
            }
        }
        if (d == -1) {
            throw new ArrayIndexOutOfBoundsException("Time is not in bounds");
        }
        double xDerivative = 3*x_interpolant[d][0]*Math.pow(time,2) + 2*x_interpolant[d][1]*time + x_interpolant[d][2];
        double yDerivative = 3*y_interpolant[d][0]*Math.pow(time,2) + 2*y_interpolant[d][1]*time + y_interpolant[d][2];
        double angle = Math.toDegrees(Math.atan2(yDerivative,xDerivative));
        return angle;
    }
    public static double getDistance(double ta, double tb) {
        // setup domains
        int da = -1;
        for (int t = 0; t < domain.length; t++) {
            if (ta >= domain[t][0] && ta <= domain[t][1]) {
                da = t;
            }
        }
        if (da == -1) {
            throw new ArrayIndexOutOfBoundsException("Time is not in bounds");
        }
        int db = -1;
        for (int t = 0; t < domain.length; t++) {
            if (tb >= domain[t][0] && tb <= domain[t][1]) {
                db = t;
            }
        }
        if (db == -1) {
            throw new ArrayIndexOutOfBoundsException("Time is not in bounds");
        }

        double distance = 0;
        if (da == db) {
            // distance from ta to tb
            distance = getDistancePiece(ta,tb,da);
        } else {
            // distance from ta to boundary
            distance += getDistancePiece(ta,domain[da][1],da);
            for (int a = 0; a < (db-da-1); a++) {
                // distance from boundary to boundary for equation a + da + 1
                int d = a + da + 1;
                distance += getDistancePiece(domain[d][0],domain[d][1],d);
            }
            // distance from boundary to tb
            distance += getDistancePiece(domain[db][0],tb,db);
        }
        return distance;
    }
    public static double getDistancePiece(double a, double b, int domain) {
        double n = 10000;
        double h = (b-a)/n;
        double sum = 0.5*(integral(a,domain)+integral(b,domain));
        for (int i = 1; i < n; i++) {
            double x = a+h*i;
            sum = sum + integral(x,domain);
        }
        double distance = sum * h;
        return distance;
    }
    public static double integral(double q, int domain) {
        double a = x_interpolant[domain][0];
        double b = x_interpolant[domain][1];
        double c = x_interpolant[domain][2];
        double d = y_interpolant[domain][0];
        double e = y_interpolant[domain][1];
        double f = y_interpolant[domain][2];

        double x = (3*a*q*q)+(2*b*q)+(c);
        double y = (3*d*q*q)+(2*e*q)+(f);
        double i = Math.sqrt(Math.pow(x,2)+Math.pow(y,2));
        return i;
    }
    public static double[][] getDomain() {
        return domain;
    }
    public static double[][] getInterpolantX() {
        return x_interpolant;
    }
    public static double[][] getInterpolantY() {
        return y_interpolant;
    }
}