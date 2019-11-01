import numpy as np
import math

def FindRPeaks(ecg_array, discretisation):
    length = len(ecg_array)
    R_peaks = list()
    X = ecg_array
    Y = list()
    T = list()
    for i in range(0, length-2, int(discretisation/2)):
        for j in range(i, i + int(discretisation/2) - 2):
            if not(max(X[i:i+2*discretisation]) > 
                   0.8*min(abs(X[i:i+2*discretisation]))):
                X[j] = -X[j]
                T.append(0.8*min(X[i:i+2*discretisation]))
            else:
                X[j] = X[j]
                T.append(max(X[i:i+2*discretisation]))
            Y.append(abs(X[j+2]-X[j]))
        Y_max = max(Y)
        Y.clear()
        for j in range(i, i + int(discretisation/2)):
            if(X[j] > 0.75*Y_max):
                R_peaks.append(j)
    return np.asarray(R_peaks)