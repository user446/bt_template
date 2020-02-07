import wfdb
import time
import argparse
import numpy

Non_Beat_list = ["[","!","]","x","(",")","p","t","u","`","'","^","|","~","+","s","T","*","D","=",'"',"@"]
Beat_list = ["N","L","R","B","A","a","J","S","V","r","F","e","j","n","E","/","f","Q","?"]
Aux_notes = [["(AB",     "Atrial bigeminy"],
            ["(AFIB",   "Atrial fibrillation"],
            ["(AFL",    "Atrial flutter"],
            ["(B",		"Ventricular bigeminy"],
            ["(BII",    "2° heart block"],
            ["(IVR",    "Idioventricular rhythm"],
            ["(N",      "Normal sinus rhythm"],
            ["(NOD",    "Nodal (A-V junctional) rhythm"],
            ["(P",		"Paced rhythm"],
            ["(PREX",   "Pre-excitation (WPW)"],
            ["(SBR",    "Sinus bradycardia"],
            ["(SVTA",   "Supraventricular tachyarrhythmia"],
            ["(T",      "Ventricular trigeminy"],
            ["(VFL",    "Ventricular flutter"],
            ["(VT",		"Ventricular tachycardia"]]

def main(args):
    
    try:
       l = int(args.length)*360
       b = int(args.begin)*360
    except:
        if isinstance(args.length, str) and args.length == 'n':
            l = None
        else:
            raise RuntimeError
        if not isinstance(int(args.begin), int):
            raise RuntimeError

    record = wfdb.rdrecord(args.input, sampfrom=b, sampto=b+l)
    annotation = wfdb.rdann(args.input, 'atr', sampfrom=b, sampto=b+l)
    
    txt = open("ecg_array.txt", "w+")
    f = open("ecg_array.c","w+")
    f.flush()
    
    f.write("const short ECG_SAMPLES[%d] = {\n" % record.sig_len)
    
    full_data = record.p_signal[:,args.channel]
    for dt in full_data:
        f.write("%d,\n" % int(dt*args.multiply))
        txt.write("%d,\n" % int(dt*args.multiply))
    f.write("};\r\n")
    
    f.close()
    txt.close()
    
    txt = open("rpeak_array.txt", "w+")
    f = open("rpeak_array.c","w+")
    f.flush()
    f.write('#include "main.h"\r\n')
    f.write("const int RPEAK_LENGTH = %d;\n" % annotation.sample.size)
    
    f.write("const int RpeakSamples[RPEAK_LENGTH] = {\n")
    marks = annotation.sample[:]
    i = 0
    error_shift = 0
    for dt in marks:
        if annotation.symbol[i] in Non_Beat_list:
            #if annotation.symbol[i] is '+':
            f.write("//%d - describes beat change: %s, mark %s\n" % (int(dt - b), annotation.aux_note[i], annotation.symbol[i]))
            txt.write("#%d - describes beat change: %s, mark %s\n" % (int(dt - b), annotation.aux_note[i], annotation.symbol[i]))
            i = i + 1
            continue
        
        if(args.adjust):
            if full_data[dt - b] > 0:
                if abs(full_data[dt - b + 1]) > abs(full_data[dt - b]):
                    while abs(full_data[dt - b + error_shift + 1]) > abs(full_data[dt - b + error_shift]):
                        error_shift = error_shift + 1
                    dt = dt + error_shift
                    f.write("%d, //%d, mark:%s\n" % (int(dt - b), error_shift, annotation.symbol[i]))
                    txt.write("%d, #%d, mark:%s\n" % (int(dt - b), error_shift, annotation.symbol[i]))
                    error_shift = 0
                else:
                    f.write("%d,\n" % int(dt - b))
                    txt.write("%d,\n" % int(dt - b))
            else:
                if abs(full_data[dt - b + 1]) < abs(full_data[dt - b]):
                    while abs(full_data[dt - b + error_shift + 1]) < abs(full_data[dt - b + error_shift]):
                        error_shift = error_shift - 1
                    dt = dt + error_shift
                    f.write("%d, //%d, mark:%s\n" % (int(dt - b), error_shift, annotation.symbol[i]))
                    txt.write("%d, #%d, mark:%s\n" % (int(dt - b), error_shift, annotation.symbol[i]))
                    error_shift = 0
                else:
                    f.write("%d,\n" % int(dt - b))
                    txt.write("%d,\n" % int(dt - b))
        else:
            f.write("%d,\n" % int(dt - b))
            txt.write("%d,\n" % int(dt - b))
        i = i + 1
    f.write("};\n")
    f.close()
    txt.close()
        


if __name__ == "__main__":

    parser = argparse.ArgumentParser(
        description='MIDdb to .c file parser')
    parser.add_argument('-i', action='store', dest='input', type=str,
                        default='mitdb/203', help='Pass path to input file')
    parser.add_argument('-b', action='store', dest='begin', type=str, default='300',
                        help='Pass beginning timestamp in seconds, default = 0')
    parser.add_argument('-l', action='store', dest='length', type=str, default='580',
                        help='Pass time in seconds or print "n" to print all')
    parser.add_argument('-ch', action='store', dest='channel', type=int, default=0,
                        help='Pass number of channel to parse')
    parser.add_argument('-m', action='store', dest='multiply', type=float, default=1000,
                        help='Multiplier for values in ECG array')
    parser.add_argument('-j', action='store', dest='adjust', type=bool, default=True,
                        help='Adjustion of positions of R marks')
    args = parser.parse_args()
    
    main(args)