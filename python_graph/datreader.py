import wfdb
import time
import argparse
import numpy


def closestNumber(n, m) : 
    # Find the quotient 
    q = int(n / m) 
      
    # 1st possible closest number 
    n1 = m * q 
      
    # 2nd possible closest number 
    if((n * m) > 0) : 
        n2 = (m * (q + 1))  
    else : 
        n2 = (m * (q - 1)) 
      
    # if true, then n1 is the required closest number 
    if (abs(n - n1) < abs(n - n2)) : 
        return n1 
      
    # else n2 is the required closest number  
    return n2 


def main(args):
    
    try:
       l = closestNumber(int(args.length)*360, 256)
    except:
        if isinstance(args.length, str) and args.length == 'n':
            l = None
        else:
            raise RuntimeError

    record = wfdb.rdrecord(args.input, sampto=l)
    annotation = wfdb.rdann(args.input, 'atr', sampto=l)
    
    
    f = open("ecg_array.c","w+")
    f.flush()
    
    # f.write("#ifndef __ECG_SAMP__\n")
    # f.write("#define __ECG_SAMP__\r\n")
    # f.write('#include "main.h"\r\n')
    # f.write("const int PRESET_LENGTH = %d;\n" % record.sig_len)
    f.write("const short ECG_SAMPLES[%d] = {\n" %record.sig_len)
    
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
    r_peaks = annotation.sample[1:]
    for dt in r_peaks:
        f.write("%d,\n" % int(dt))
    f.write("};\n")
    # f.write("#endif\n")
    f.close()
        


if __name__ == "__main__":

    parser = argparse.ArgumentParser(
        description='MIDdb to .c file parser')
    parser.add_argument('-i', action='store', dest='input', type=str,
                        default='mitdb/100', help='Pass path to input file')
    parser.add_argument('-l', action='store', dest='length', type=str, default='1800',
                        help='Pass time in seconds or print "n" to print all')
    parser.add_argument('-ch', action='store', dest='channel', type=int, default=0,
                        help='Pass number of channel to parse')
    parser.add_argument('-j', action='store', dest='adjust', type=int, default=10000,
                        help='Pass number of channel to parse')
    args = parser.parse_args()
    
    main(args)