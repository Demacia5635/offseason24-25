package frc.robot.subsystems.shooter.utils;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;

/**
 * This class represents a look-up table for linear interpolation.
 * 
 * @author (Please add your name here)
 * @version (Please add the version number here)
 */
public class LookUpTable {

    /**The internal table data as a list of double arrays. */
    private final List<double[]> table;

    /**The number of columns in the table (one more than the actual number of interpolated values). */
    private final int size;

    /**
     * Creates a new empty look-up table with the specified size.
     *
     * @param size the number of interpolated values
     * @throws IllegalArgumentException if size is less than 2
     */
    public LookUpTable(int size) {
        if (size < 2) {
            throw new IllegalArgumentException("Size must be at least 2.");
        }
        table = new ArrayList<>();
        this.size = size + 1;
    }

    /**
     * Creates a new look-up table from an existing 2D array of data.
     *
     * @param table the 2D array containing the look-up table data
     * @throws IllegalArgumentException if any row in the table has a different length than the others
     */
    public LookUpTable(double[][] table) {
        this.table = new ArrayList<>();
        sort(table);
        for (double[] row : table) {
            this.table.add(row);
        }
        size = table[0].length;

        for (int i = 1; i < table.length; i++) {
            if (table[i].length != size) {
                throw new IllegalArgumentException("All rows in the table must have the same length.");
            }
        }
    }

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
     * Adds a new row to the table.
     *
     * @param row the row to be added, must have the same size as the existing rows
     * @throws IllegalArgumentException if the row size doesn't match the table size
     */
    public void add(double... row) throws IllegalArgumentException {
        if (row.length != size) {
            throw new IllegalArgumentException("Size of new row (" + row.length + ") does not match row size of: " + size);
        }
        table.add(row);
    }

    /**
     * Interpolates and returns an array of values based on the given input value.
     *
     * @param value the input value
     * @return an array of interpolated values, even if the input value falls outside the table's range
     */
    public double[] get(double value) {
        double[] ans = new double[size - 1];
        ans[0] = value;

        int length = table.size();
        if (length == 0) {
            return ans;
        }

        if (length == 1) {
            double[] row = table.get(0);
            for (int i = 1; i < size; i++) {
                ans[i - 1] = row[i];
            }
            return ans;
        }

        double[] lower, upper;
        int i;

        i = 0;
        for (i = 0; i < length && value > table.get(i)[0]; i++);

        if (i == 0) {
            lower = table.get(0);
            upper = table.get(1);
        } else if (i == length) {
            lower = table.get(length - 2);
            upper = table.get(length - 1);
        } else {
            lower = table.get(i - 1);
            upper = table.get(i);
        }

        for (int j = 1; j < size; j++) {
            ans[j - 1] = lower[j] + (value - lower[0]) * (upper[j] - lower[j]) / (upper[0] - lower[0]);
        }

        return ans;
    }
}
