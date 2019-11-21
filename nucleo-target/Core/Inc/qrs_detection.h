#ifndef QRS_DETECTION_H
#define QRS_DETECTION_H

#define MARK_NO_QRS (0)
#define MARK_QRS    (1)

int DetectQrsPeaks(float const* signal, int size, char* result, float rate);

#endif
