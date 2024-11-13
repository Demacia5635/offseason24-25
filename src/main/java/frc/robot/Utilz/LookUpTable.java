package frc.robot.Utilz;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;

/**
 * This class represents a look-up table for linear interpolation.
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
    public LookUpTable(int size) throws IllegalArgumentException {
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
    public LookUpTable(double[][] table) throws IllegalArgumentException {
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
        
        /* checks if the value is or bigger than the biggest point */
        if (value >= table.get(table.size() - 1)[0]) {
            double[] ans = new double[size -1];
            for (int j = 1; j < size; j++) {
                ans[j-1] = table.get(table.size() - 1)[j];
            }
            
            return ans;

        /* checks if the value is or smaller than the smallest point */
        } else if (value <= table.get(0)[0]) {
            double[] ans = new double[size - 1];
            for (int j = 1; j < size; j++) {
                ans[j - 1] = table.get(0)[j];
            }

            return ans;
        }

        /* checks what is the after and before points of the value */
        // int i = 0;
        // for (i = 0; i < table.size() && table.get(i)[0] < value; i++);

        int left = 0;
        int right = table.size() - 1;
        int mid = left;
        
        while (left < right) {
            mid = left + (right - left) / 2;

            if (table.get(mid)[0] < value) {
                left = mid + 1;
            } else if (table.get(mid)[0] > value) {
                right = mid;
            } else {
                left = mid;
                break;
            }
        }

        /* assign the after and before points */
        double[] after, before;
        after = table.get(left);
        before = table.get(left - 1);
        
        /* set all of the var to be at the right var for the value */
        double[] ans = new double[size - 1];
        for (int j = 1; j < size; j++) {
            ans[j-1] = ((after[j] - before[j]) / (after[0] - before[0])) * value + (-((after[j] - before[j]) / (after[0] - before[0])) * after[0] + after[j]);
        }
     
        /* returns the right var for the value as a arr of double */
        return ans;
    }
}