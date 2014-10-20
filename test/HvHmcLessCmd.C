#include <strings.h>

#ifndef  _HVHMCLESSCMD_H
    #include "HvHmcLessCmd.H"
#endif

HvHmcLessCmd::HvHmcLessCmd(Target target)
:   xCorrelationToken(NULL)
,   xTypeInd(Request)
,   xRspInd(RspExpected)
,   xRsvd1(0)
,   xRsvd2(0)
,   xRsvd3(0)
,   xOpcode(0)
,   xHeaderLength(HvHmcCmdHdrLen)
,   xRc(Good)
,   xDataLength(0)
,   xMessageLength(HvHmcCmdHdrLen)

{
    xDesc[0] = 0xDE;
    xDesc[1] = 0xCA;
    xDesc[2] = 0xFF;
    bzero(xPayload,HvHmcCmdMaxSize-HvHmcCmdHdrLen);
    xTarget = target;

}

void  HvHmcLessCmd::setDataLen(uint32 len)
{
    xDataLength = len;
    xMessageLength = len + xHeaderLength;

}


void HvHmcLessCmd::dumpHex()
{
    unsigned char* data = (unsigned char*) this;
    uint16 numberOfBytes = this->xMessageLength;
    FILE* outFilePtr = stdout;

    int i;
    for (i=0;i<numberOfBytes;i++) {
        fprintf(outFilePtr," %02x", data[i]);
        if (((i % 16) == 15)) {
            fprintf(outFilePtr, "  ");
            for (int c=i-15;c<=i;c++) {
                if ((data[c] >= 0x20) && (data[c] <= 0x7E)) {
                    fputc(data[c], outFilePtr);
                } else {
                    fputc('.', outFilePtr);
                }
            }
            fprintf(outFilePtr, "\n");
        }
    }
    int bytesOnLastLine = (i % 16);

    if (bytesOnLastLine != 0) {
        for (int j=0;j<(16 - bytesOnLastLine)*3 + 2;j++) {
            fputc(' ', outFilePtr);
        }
        for (int c=i-bytesOnLastLine;c<i;c++) {
            if ((data[c] >= 0x20) && (data[c] <= 0x7E)) {
                fputc(data[c], outFilePtr);
            } else {
                fputc('.', outFilePtr);
            }
        }



    }
    fprintf(outFilePtr, "\n");
}
