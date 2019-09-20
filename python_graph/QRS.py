import numpy as np
import math

# преобразование длинны отрезка с сигналом ЭКГ


def length_transform(ecg, length):
    lgth = ecg.shape[0]  # возвращает длину массива ЭКГ
    diff = np.zeros(lgth)
    # добавляет слева и справа массива его значения на границах
    ecg = np.pad(ecg, length, 'edge')
    for i in range(lgth):
        temp = ecg[i:i+length*2+1]
        left = temp[length] - temp[0]
        right = temp[length] - temp[-1]
        diff[i] = min(left, right)
        # если значения в массиве меньше нуля, то просто приравниваем к нулю
        diff[diff < 0] = 0
    return np.multiply(diff, diff)

# интегрирование сигнала ЭКГ заданной длинны


def integrate(ecg, length):
    lgth = ecg.shape[0]
    integrate_ecg = np.zeros(lgth)
    ecg = np.pad(ecg, math.ceil(length/2), 'symmetric')
    for i in range(lgth):
        # интеграл по формуле трапеций
        integrate_ecg[i] = np.sum(ecg[i:i+length])/length
    return integrate_ecg

# нахождение пиков


def find_peak(data, length):
    lgth = data.shape[0]
    true_peaks = list()
    for i in range(lgth-length+1):
        temp = data[i:i+length]
        var = np.var(temp)
        if var < 5e-15:
            continue
        index = int((length-1)/2)
        peak = True
        for j in range(index):
            if temp[index-j] <= temp[index-j-1] or temp[index+j] <= temp[index+j+1]:
                peak = False
                break
        if peak is True:
            true_peaks.append(int(i+(length-1)/2))
    return np.asarray(true_peaks)


def find_R_peaks(ecg, peaks, length):
    num_peak = peaks.shape[0]
    R_peaks = list()
    for index in range(num_peak):
        i = peaks[index]
        if i-2*length > 0 and i < ecg.shape[0]:
            temp_ecg = ecg[i-2*length:i]
            R_peaks.append(int(np.argmax(temp_ecg)+i-2*length))
    return np.asarray(R_peaks)


def find_S_point(ecg, R_peaks):
    num_peak = R_peaks.shape[0]
    S_point = list()
    for index in range(num_peak):
        i = R_peaks[index]
        cnt = i
        if cnt+1 >= ecg.shape[0]:
            break
        while ecg[cnt] > ecg[cnt+1]:
            cnt += 1
            if cnt >= ecg.shape[0]:
                break
        S_point.append(cnt)
    return np.asarray(S_point)


def find_Q_point(ecg, R_peaks):
    num_peak = R_peaks.shape[0]
    Q_point = list()
    for index in range(num_peak):
        i = R_peaks[index]
        cnt = i
        if cnt-1 < 0:
            break
        while ecg[cnt] > ecg[cnt-1]:
            cnt -= 1
            if cnt < 0:
                break
            Q_point.append(cnt)
    return np.asarray(Q_point)


def ECG_QRS_detect(ecg, fs):
    ecg = ecg - np.mean(ecg)
    ecg_length_transform = length_transform(ecg, int(fs/20))

    length = int(fs/8)
    ecg_integrate = integrate(ecg_length_transform, length)/length
    length = int(fs/6)
    ecg_integrate = integrate(ecg_integrate, length)
    length = int(fs/36)
    ecg_integrate = integrate(ecg_integrate, length)
    length = int(fs/72)
    ecg_integrate = integrate(ecg_integrate, length)

    peaks = find_peak(ecg_integrate, int(fs/10))
    R_peaks = find_R_peaks(ecg, peaks, int(fs/40))
    S_point = find_S_point(ecg, R_peaks)
    Q_point = find_Q_point(ecg, R_peaks)
    return R_peaks, S_point, Q_point
