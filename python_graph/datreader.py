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
       l = closestNumber(int(args.length)*360, 255) + 1
    except:
        if isinstance(args.length, str) and args.length == 'n':
            l = None
        else:
            raise RuntimeError

    record = wfdb.rdrecord(args.input, sampto=l)
    annotation = wfdb.rdann('mitdb/100', 'atr', sampto=l)
    
    
    f = open(args.output,"w+")
    f.flush()
    
    f.write("#ifndef __ECG_SAMP__\n")
    f.write("#define __ECG_SAMP__\r\n")
    f.write("#define PRESET_LENGTH %d\n" % record.sig_len)
    f.write("#define RPEAK_LENGTH %d\n" % annotation.sample.size)
    f.write("int ecg_samples[PRESET_LENGTH] = {\n")
    
    full_data = record.p_signal[:,args.channel]
    for dt in full_data:
        f.write("%d,\n" % int(dt*args.adjust))
    f.write("};\r\n")
    
    f.write("int RpeakSamples[RPEAK_LENGTH] = {\n")
    r_peaks = annotation.sample
    for dt in r_peaks:
        f.write("%d,\n" % int(dt))
    f.write("};\n")
    f.write("#endif\n")
    f.close()
        


if __name__ == "__main__":

    parser = argparse.ArgumentParser(
        description='MIDdb to .c file parser')
    parser.add_argument('-i', action='store', dest='input', type=str,
                        default='mitdb/100', help='Pass path to input file')
    parser.add_argument('-o', action='store', dest='output', type=str,
                        default='ecg_array.h', help='Pass path to output file')
    parser.add_argument('-l', action='store', dest='length', type=str, default='60',
                        help='Pass time in seconds or print "n" to print all')
    parser.add_argument('-ch', action='store', dest='channel', type=int, default=0,
                        help='Pass number of channel to parse')
    parser.add_argument('-j', action='store', dest='adjust', type=int, default=1000000,
                        help='Pass number of channel to parse')
    args = parser.parse_args()
    
    main(args)