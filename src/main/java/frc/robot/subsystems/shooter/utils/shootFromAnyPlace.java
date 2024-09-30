// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.utils;

import java.util.Arrays;
import java.util.Comparator;

/**util that will store the entire lookup table and can send back the angle and the pow needed by dis */
public class shootFromAnyPlace {
    private double[][] lookupTable = {
        {0,0,0},{0,0,0},{0,0,0}
    };

    /**creates a new shootFromAnyPlace class */
    public shootFromAnyPlace(){}

    /**
     * sort an array
     * @param arr the wanted sorted array in double[][]
     */
    private void sort(double[][] arr){
        Arrays.sort(arr, new Comparator<double[]>() {
            @Override
            public int compare(double[] entry1, double[] entry2){
                return Double.compare(entry1[0], entry2[0]);
            }
        });
    }

    /**
     * add to an array an element
     * @param arr the wanted array
     * @param element the wanted element
     * @return the arr with the element inside
     */
    private double[][] addIndex(double[][] arr, double[] element){
        double[][] newArr = new double[arr.length + 1][];
        for (int i=0; i < arr.length; i++){
            newArr[i] = arr[i];
        }
        newArr[arr.length] = element;
        return newArr;
    }

    /**
     * get the wanted angle base on the dis
     * @param dis the dis the chassis is currently in
     * @return the wanted angle to shoot perfect for the speakr
     */
    public double getAngle(double dis){

        /*checkes if the dis is already in the lookup table */
        for (double[] i : lookupTable) {
            if  (i[0] == dis){
                return i[1];
            }
        }
        
        /*set up var */
        double[] top = {0,0};
        double[] bottom = {0,0};
        double m;
        double b;
        double[] x = {dis, 0, 0};
        double[][] copy = addIndex(lookupTable, x);
        sort(copy);

        /* get the closest var from the current dis */
        for (int i=0; i < copy.length; i++){
            if (copy[i][0] == dis){
                bottom = copy[i==0 ? 0 : i-1];
                top = copy[i==copy.length-1 ? -1 : i+1];
            }
        }

        /*makes a y=mx+b function to find the right angle to shoot */
        m = (bottom[1] - top[1]) / (bottom[0] - top[0]);
        b = -1*(m*bottom[0] - bottom[1]);

        return m*dis + b;
    }


    /**
     * get the wanted pow based on the dis
     * @param dis the dis the chassis is currently in
     * @return the wanted pow to shoot perfect for the speakr
     */
    public double getPow(double dis){

        /*checkes if the dis is already in the lookup table */
        for (double[] i : lookupTable) {
            if  (i[0] == dis){
                return i[2];
            }
        }
        
        /*set up var */
        double[] top = {0,0,0};
        double[] bottom = {0,0,0};
        double m;
        double b;
        double[] x = {dis, 0 , 0};
        double[][] copy = addIndex(lookupTable, x);
        sort(copy);

        /* get the closest var from the current dis */
        for (int i=0; i < copy.length; i++){
            if (copy[i][0] == dis){
                bottom = copy[i==0 ? 0 : i-1];
                top = copy[i==copy.length-1 ? -1 : i+1];
            }
        }

        /*makes a y=mx+b function to find the right angle to shoot */
        m = (bottom[2] - top[2]) / (bottom[0] - top[0]);
        b = -1*(m*bottom[0] - bottom[2]);

        return m * dis + b;
    }
}