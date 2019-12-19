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
    except:
        if isinstance(args.length, str) and args.length == 'n':
            l = None
        else:
            raise RuntimeError

    record = wfdb.rdrecord(args.input, sampto=l)
    annotation = wfdb.rdann(args.input, 'atr', sampto=l)
    
    
    f = open("ecg_array.c","w+")
    f.flush()
    
    f.write("const short ECG_SAMPLES[%d] = {\n" % record.sig_len)
    
    full_data = record.p_signal[:,args.channel]
    for dt in full_data:
        f.write("%d,\n" % int(dt*args.adjust))
    f.write("};\r\n")
    
    f.close()
    
    f = open("rpeak_array.c","w+")
    f.flush()
    f.write('#include "main.h"\r\n')
    f.write("const int RPEAK_LENGTH = %d;\n" % annotation.sample.size)
    
    f.write("const int RpeakSamples[RPEAK_LENGTH] = {\n")
    marks = annotation.sample[:]
    i = 0
    for dt in marks:
        if annotation.symbol[i] in Non_Beat_list:
            if annotation.symbol[i] is '+':
                f.write("//%d - describes beat change: %s\n" % (int(dt), annotation.aux_note[i]))
            i = i + 1
            continue
        f.write("%d,\n" % int(dt))
        i = i + 1
    f.write("};\n")
    f.close()
        


if __name__ == "__main__":

    parser = argparse.ArgumentParser(
        description='MIDdb to .c file parser')
    parser.add_argument('-i', action='store', dest='input', type=str,
                        default='mitdb/100', help='Pass path to input file')
    parser.add_argument('-l', action='store', dest='length', type=str, default='480',
                        help='Pass time in seconds or print "n" to print all')
    parser.add_argument('-ch', action='store', dest='channel', type=int, default=0,
                        help='Pass number of channel to parse')
    parser.add_argument('-j', action='store', dest='adjust', type=int, default=10000,
                        help='Pass number of channel to parse')
    args = parser.parse_args()
    
    main(args)