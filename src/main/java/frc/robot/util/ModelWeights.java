package frc.robot.util;

public class ModelWeights {
  public static double[][] weights1 = {
    {
      0.0457374,
      -0.3168908,
      0.51027256,
      0.01145879,
      0.5272291,
      0.02359425,
      0.04945777,
      0.07793193,
      0.08562946,
      -0.32115534,
      -0.2408068,
      0.26228553,
      -0.17997377,
      0.03408399,
      0.29043642,
      -0.25590625
    },
    {
      0.33659318, -0.14726357, -0.20459077, -0.22826664, 0.30141857, 0.11818106,
      0.18646216, 0.07916409, -0.05622721, -0.5128752, -0.7953316, 0.15174817,
      0.08826113, -0.61534995, -0.08751829, 0.38782975
    },
    {
      -0.5774205,
      -0.05775994,
      0.14927939,
      -0.10137261,
      0.1146134,
      0.5453778,
      0.27717102,
      -0.1865191,
      -0.09594834,
      0.02212042,
      0.14320683,
      0.10416884,
      -0.04239413,
      -0.00370209,
      0.44472063,
      -0.02290833
    },
    {
      -0.21989855, -0.23798981, 0.35964465, -0.7451136, 0.31031188, -0.0732232,
      -0.06084732, -0.77193063, -0.4225396, -0.17253613, 0.21201123, 0.28133544,
      -0.11934469, -0.9775352, 0.02208214, 0.14827466
    },
    {
      0.4499242,
      0.62746716,
      0.4351888,
      0.29777002,
      -0.63607204,
      0.42014396,
      -0.0270689,
      -0.23012504,
      -0.35641503,
      0.4112539,
      -0.13865532,
      0.38322654,
      0.71487415,
      0.01194455,
      0.45676374,
      0.88612324
    },
    {
      0.0181332,
      0.33283612,
      -0.02525444,
      0.5388179,
      -0.15092161,
      -0.30843768,
      0.4074096,
      0.706234,
      0.20086062,
      0.17678876,
      -0.44153023,
      0.11219154,
      -0.10907228,
      0.15358844,
      0.05605988,
      -0.499536
    },
    {
      0.32017842,
      -0.28838167,
      0.3101882,
      0.6421666,
      -0.16622017,
      -0.04429009,
      -0.09428831,
      0.20977868,
      0.06882489,
      -0.2371534,
      -0.28965685,
      -0.24413559,
      -0.68164533,
      0.23283438,
      0.04091965,
      -0.06461232
    },
    {
      -0.7563242,
      0.45103458,
      -0.18121493,
      -0.04370422,
      0.09630562,
      -0.4717242,
      0.18063974,
      0.50838256,
      -0.2839626,
      -0.12190229,
      0.02777763,
      0.02835446,
      0.19145286,
      0.36584005,
      -0.04535041,
      -0.58086
    }
  };

  public static double[][] biases1 = {
    {
      -0.08015949,
      -0.09423959,
      -0.3893247,
      0.23721226,
      0.17550898,
      -0.14201008,
      0.21037602,
      -0.30728155,
      0.,
      -0.0729275,
      0.39143747,
      -0.12433809,
      0.09537148,
      0.16059443,
      -0.11288227,
      -0.13509421
    }
  };

  public static double[][] weights2 = {
    {-0.9330981, -1.1404595},
    {0.64817846, -0.00557974},
    {-0.5446137, 0.3739858},
    {0.5635056, 0.7775643},
    {0.52912474, 0.47469315},
    {-0.69655716, -0.8386202},
    {-0.48737505, 0.43157616},
    {-0.53041, -0.8992263},
    {-0.4492076, 0.551512},
    {0.35254428, -0.1479326},
    {0.26975924, 0.8351715},
    {0.2963063, -0.4675373},
    {-0.6630343, 0.55918235},
    {-0.43375087, -0.46690738},
    {0.36194423, -0.3667702},
    {1.0456718, 0.98170644}
  };
  public static double[][] biases2 = {{0.09335279, 0.21503644}};

  public static double[][] matrixMultiply(double[][] A, double[][] B) {
    int rowsA = A.length, colsA = A[0].length;
    int rowsB = B.length, colsB = B[0].length;

    if (colsA != rowsB) {
      throw new IllegalArgumentException("Matrix dimensions do not match for multiplication.");
    }

    double[][] result = new double[rowsA][colsB];

    for (int i = 0; i < rowsA; i++) {
      for (int j = 0; j < colsB; j++) {
        for (int k = 0; k < colsA; k++) {
          result[i][j] += A[i][k] * B[k][j];
        }
      }
    }

    return result;
  }

  public static double[][] matrixAdd(double[][] A, double[][] B) {
    int rows = A.length, cols = A[0].length;

    if (rows != B.length || cols != B[0].length) {
      throw new IllegalArgumentException("Matrix dimensions do not match for addition.");
    }

    double[][] result = new double[rows][cols];

    for (int i = 0; i < rows; i++) {
      for (int j = 0; j < cols; j++) {
        result[i][j] = A[i][j] + B[i][j];
      }
    }

    return result;
  }

  public static double[][] applyReLU(double[][] matrix) {
    int rows = matrix.length, cols = matrix[0].length;
    double[][] result = new double[rows][cols];

    for (int i = 0; i < rows; i++) {
      for (int j = 0; j < cols; j++) {
        result[i][j] = Math.max(0, matrix[i][j]);
      }
    }

    return result;
  }
}
