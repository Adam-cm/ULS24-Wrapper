// Copyright 2014-2023, Anitoa Systems, LLC
// All rights reserved

#include "TrimReader.h"
#include <fstream>
#include <sstream>
#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstring>
#include <cstdint>

// Define important algorithm flags to match Windows implementation
#define SAW_TOOTH2     // Newer Sawtooth algorithm. Use 2 pass low byte correction
#define NON_CONTIGUOUS // Support for non-contiguous conversion

// Helper: trim left - C++ standard library implementation
static void trimLeft(std::string& s, const std::string& delimiters) {
    s.erase(s.begin(), std::find_if(s.begin(), s.end(), [&](unsigned char ch) {
        return delimiters.find(ch) == std::string::npos;
        }));
}

// Helper: find first of any delimiter - matches Windows implementation
static size_t findOneOf(const std::string& s, const std::string& delimiters) {
    return s.find_first_of(delimiters);
}

// External variables 
extern uint8_t TxData[];
extern uint8_t RxData[];
extern int chan_num;
extern int Continue_Flag;

// EEPROM buffer - matching Windows implementation size
uint8_t EepromBuff[16 + 4 * NUM_EPKT][EPKT_SZ + 1];  // 16 pages maximum - enough for 16 well 4 channel

// CTrimNode implementation
CTrimNode::CTrimNode() { Initialize(); }

void CTrimNode::Initialize() {
    for (int i = 0; i < TRIM_IMAGER_SIZE; i++) {
        kb[i][0] = 1;   // k
        kb[i][1] = 0;   // b
        kb[i][2] = 0;   // k2
        kb[i][3] = 0;   // b2
        kb[i][4] = 0;   // c
        kb[i][5] = 0;   // h

        fpn[0][i] = 0;
        fpn[1][i] = 0;
        tempcal[i] = (i == 0) ? 1 : 0;
    }

    rampgen = 0x88;
    range = 0xf;
    auto_v20[0] = 0x8;
    auto_v20[1] = 0xa;
    auto_v15 = 0x8;
    version = 0x0;
    tbuff_size = 0;
    tbuff_rptr = 0;
}

// CTrimReader implementation
CTrimReader::CTrimReader()
    : curNode(nullptr), NumNode(0), WordIndex(0), MaxWord(0), fileLoaded(false) {
}

CTrimReader::~CTrimReader() {}

int CTrimReader::Load(const std::string& fn) {
    std::ifstream inFile(fn);
    if (!inFile) {
        fileLoaded = false;
        return -1;
    }
    fileLoaded = true;

    std::stringstream buffer;
    buffer << inFile.rdbuf();
    std::string FileBuf = buffer.str();

    const std::string delimit = ", \t\r\n";
    int i = 0;
    trimLeft(FileBuf, delimit);

    // Parse input file into word buffer
    while (i < TRIM_MAX_WORD) {
        size_t ep = findOneOf(FileBuf, delimit);
        if (ep == std::string::npos)
            break;
        WordBuf[i] = FileBuf.substr(0, ep);
        FileBuf = FileBuf.substr(ep);
        trimLeft(FileBuf, delimit);
        ++i;
    }

    MaxWord = i;
    return 0;
}

int CTrimReader::GetWord() {
    if (WordIndex < MaxWord) {
        CurWord = WordBuf[WordIndex];
        ++WordIndex;
    }
    else {
        CurWord.clear();
    }
    return WordIndex;
}

int CTrimReader::Match(const std::string& s) {
    return CurWord == s ? 1 : 0;
}

void CTrimReader::Parse() {
    std::string Name;
    int i = 0;

    if (!fileLoaded)
        return;

    for (;;) {
        if (GetWord() == MaxWord)
            break;

        if (Match("DEF")) {
            GetWord();
            Name = CurWord;

            GetWord();
            if (Match("{")) {
                curNode = Node + i;
                i++;
                curNode->name = Name;
                ParseNode();
            }
            else break;
        }
        else break;
    }

    NumNode = i;
}

void CTrimReader::ParseNode() {
    if (!fileLoaded)
        return;

    for (;;) {
        if (GetWord() == MaxWord)
            break;

        if (Match("Kb")) {
            GetWord();
            if (Match("{")) {
                ParseMatrix();
                GetWord();
                if (!Match("}"))
                    return;
            }
            else return;
        }
        else if (Match("Fpn_lg")) {
            GetWord();
            if (Match("{")) {
                ParseArray(0);
                GetWord();
                if (!Match("}"))
                    return;
            }
            else return;
        }
        else if (Match("Fpn_hg")) {
            GetWord();
            if (Match("{")) {
                ParseArray(1);
                GetWord();
                if (!Match("}"))
                    return;
            }
            else return;
        }
        else if (Match("Temp_calib")) {
            GetWord();
            if (Match("{")) {
                ParseArray(2);
                GetWord();
                if (!Match("}"))
                    return;
            }
            else return;
        }
        else if (Match("Rampgen")) {
            GetWord();
            if (Match("{")) {
                ParseValue(2);
                GetWord();
                if (!Match("}"))
                    return;
            }
            else return;
        }
        else if (Match("AutoV20_lg")) {
            GetWord();
            if (Match("{")) {
                ParseValue(0);
                GetWord();
                if (!Match("}"))
                    return;
            }
            else return;
        }
        else if (Match("AutoV20_hg")) {
            GetWord();
            if (Match("{")) {
                ParseValue(1);
                GetWord();
                if (!Match("}"))
                    return;
            }
            else return;
        }
        else if (Match("AutoV15")) {
            GetWord();
            if (Match("{")) {
                ParseValue(3);
                GetWord();
                if (!Match("}"))
                    return;
            }
            else return;
        }
        else if (Match("}")) {
            return;
        }
        else
            return;
    }
}

void CTrimReader::ParseMatrix() {
    for (int i = 0; i < TRIM_IMAGER_SIZE; i++) {
        for (int j = 0; j < 4; j++) {
            if (GetWord() == MaxWord)
                break;
            try {
                curNode->kb[i][j] = std::stof(CurWord);
            }
            catch (...) {
                curNode->kb[i][j] = 0.0;
            }
        }
    }
}

void CTrimReader::ParseArray(int gain) {
    for (int i = 0; i < 12; i++) {
        GetWord();
        try {
            if (gain == 2)
                curNode->tempcal[i] = std::stof(CurWord);
            else
                curNode->fpn[gain][i] = std::stof(CurWord);
        }
        catch (...) {
            if (gain == 2)
                curNode->tempcal[i] = 0.0;
            else
                curNode->fpn[gain][i] = 0.0;
        }
    }
}

// gain: 0, 1 - auto_v20[0, 1]; 2: rampgen; 3: auto_v15
void CTrimReader::ParseValue(int gain) {
    GetWord();
    std::string word = CurWord;
    std::transform(word.begin(), word.end(), word.begin(), ::tolower);
    size_t p = word.find("0x");
    unsigned int val = 0;

    if (p != std::string::npos) {
        try {
            val = std::stoul(word.substr(p + 2), nullptr, 16);
        }
        catch (...) {
            val = 0;
        }
    }
    else {
        try {
            val = std::stoul(word, nullptr, 10);
        }
        catch (...) {
            val = 0;
        }
    }

    if (gain == 2)
        curNode->rampgen = val;
    else if (gain == 3)
        curNode->auto_v15 = val;
    else
        curNode->auto_v20[gain] = val;
}

// Constants for ADC processing
#define DARK_LEVEL 100
#define DARK_MANAGE

// NumData =  "Column Number"
// pixelNum = "Frame Size"

// ADC correction with floating-point implementation
int CTrimReader::ADCCorrection(int NumData, uint8_t HighByte, uint8_t LowByte, int pixelNum, int PCRNum, int gain_mode, int* flag)
{
    int hb, lb, lbc;
    int hbln, lbp, hbhn;
    bool oflow = false, uflow = false;

    double ioffset = 0;
    int result;

    hb = (int)HighByte;

    int nd = 0;
    if (pixelNum == 12) nd = NumData;
    else nd = NumData >> 1;

    ioffset = Node[PCRNum - 1].kb[nd][0] * (double)hb + Node[PCRNum - 1].kb[nd][1];

#ifdef NON_CONTIGUOUS
    if (hb >= 128) {
        ioffset += Node[PCRNum - 1].kb[nd][3];
    }
#endif

    hbln = hb % 16;
    hbhn = hb / 16;

#ifdef SAW_TOOTH
    ioffset += Node[PCRNum - 1].kb[nd][2] * (hbln - 7);
#endif

    lb = (int)LowByte;
    lbc = lb + (int)ioffset;

#ifdef SAW_TOOTH2 // Use lbc, not hbln to calculate sawtooth correction, as hbln tends to be a little unstable
    ioffset += Node[PCRNum - 1].kb[nd][2] * ((double)lbc - 127) * (1 - (double)hb / 400) / 16; // 12/19/2016 modification
    lbc = lb + (int)ioffset; // re-calc lbc, 2 pass algorithm
#endif

    lbp = hbln * 16 + 7;

    if (lbc > 255) lbc = 255;
    else if (lbc < 0) lbc = 0;

    int lbpc = lbp - (int)ioffset; // lpb - ioffset: low byte predicted from the high byte low nibble BEFORE correction
    int qerr = lbp - lbc; // if the lbc is correct, this would be the quantization error. If it is too large, maybe lb was the saturated "stuck" version

    if (lbpc > 255 + 20) { // We allow some correction error, because hbln may have randomly flipped.
        oflow = true; *flag = 1;
    }
    else if (lbpc > 255 && qerr > 28) { // Allow tolerance because hbln may have drifted
        oflow = true; *flag = 2;
    }
    else if (lbpc > 191 && qerr > 52) {
        oflow = true; *flag = 3;
    }
    else if (qerr > 96) {
        oflow = true; *flag = 4;
    }
    else if (lbpc < -20) {
        uflow = true; *flag = 5;
    }
    else if (lbpc < 0 && qerr < -28) {
        uflow = true; *flag = 6;
    }
    else if (lbpc < 64 && qerr < -52) {
        uflow = true; *flag = 7;
    }
    else if (qerr < -96) {
        uflow = true; *flag = 8;
    }
    else {
        *flag = 0;
    }

    if (oflow || uflow) {
        result = hb * 16 + 7;
    }
    else {
        result = hbhn * 256 + lbc;
    }

#ifdef DARK_MANAGE
    if (!gain_mode)
        result += -(int)(Node[PCRNum - 1].fpn[1][nd]) + DARK_LEVEL; // high gain
    else
        result += -(int)(Node[PCRNum - 1].fpn[0][nd]) + DARK_LEVEL; // low gain

    if (result < 0) result = 0;
#endif

    return result;
}

// Integer version of the ADC correction function
int CTrimReader::ADCCorrectioni(int NumData, uint8_t HighByte, uint8_t LowByte, int pixelNum, int PCRNum, int gain_mode, int* flag)
{
    int hb, lb, lbc, hbi;
    int hbln, lbp, hbhn;
    bool oflow = false, uflow = false;

    int ioffset = 0;
    int result;

    int intmax = 32767;
    int intmax256 = 128;

    hb = (int)HighByte;
    hbln = hb % 16;
    hbhn = hb / 16;

    int nd = 0;
    if (pixelNum == 12) nd = NumData;
    else nd = NumData >> 1;

    int k, b, c, h;

    c = (int)(Node[PCRNum - 1].kbi[nd][4]);
    h = (int)(Node[PCRNum - 1].kbi[nd][5]);

    if (hb < 16) {
        k = (int)(Node[PCRNum - 1].kbi[nd][0]);
        b = (int)(Node[PCRNum - 1].kbi[nd][1]) + h / 2; // First bump raised higher
        c = c + h / 10;
    }
    else if (hb < 128) {
        k = (int)(Node[PCRNum - 1].kbi[nd][0]);
        b = (int)(Node[PCRNum - 1].kbi[nd][1]);
    }
    else {
        k = (int)(Node[PCRNum - 1].kbi[nd][2]);
        b = (int)(Node[PCRNum - 1].kbi[nd][3]);
    }

    ioffset = k * hb / intmax + b / intmax256;

    lb = (int)LowByte;
    lbc = lb + ioffset;

    if (hb > 128) {
        hbi = 128 + (hb - 128) / 2;
    }
    else {
        hbi = hb;
    }

    // Use lbc, not hbln to calculate sawtooth correction, as hbln tends to be a little jittery
    ioffset += (lbc - 128) * c * (300 - hbi) / (12 * 300 * intmax256); // 12/19/2016 modification
    lbc = lb + ioffset; // re-calc lbc, 2 pass algorithm

    if (lbc > 255) lbc = 255;
    else if (lbc < 0) lbc = 0;

    lbp = hbln * 16 + 7;

    int lbpc = lbp - ioffset; // lpb - ioffset: low byte predicted from high byte low nibble BEFORE correction
    int qerr = lbp - lbc; // quantization error

    if (lbpc > 255 + 20) { // Allow some correction error, as hbln may have randomly flipped
        oflow = true; *flag = 1;
    }
    else if (lbpc > 255 && qerr > 28) { // Allow tolerance for hbln drift
        oflow = true; *flag = 2;
    }
    else if (lbpc > 191 && qerr > 52) {
        oflow = true; *flag = 3;
    }
    else if (qerr > 96) {
        oflow = true; *flag = 4;
    }
    else if (lbpc < -20) {
        uflow = true; *flag = 5;
    }
    else if (lbpc < 0 && qerr < -28) {
        uflow = true; *flag = 6;
    }
    else if (lbpc < 64 && qerr < -52) {
        uflow = true; *flag = 7;
    }
    else if (qerr < -96) {
        uflow = true; *flag = 8;
    }
    else {
        *flag = 0;
    }

    if (oflow || uflow) {
        result = hb * 16 + 7;
    }
    else {
        result = hbhn * 256 + lbc;
    }

#ifdef DARK_MANAGE
    if (!gain_mode)
        result += -(int)(Node[PCRNum - 1].fpni[1][nd]) + DARK_LEVEL; // high gain
    else
        result += -(int)(Node[PCRNum - 1].fpni[0][nd]) + DARK_LEVEL; // low gain

    if (result < 0) result = 0;
#endif

    return result;
}

//========== Protocol Engine=================

void CTrimReader::SetV20(uint8_t v20)
{
    TxData[0] = 0xaa;      //preamble code
    TxData[1] = 0x01;      //command
    TxData[2] = 0x02;      //data length
    TxData[3] = 0x04;      //data type, date edit first byte
    TxData[4] = v20;       //real data, date edit second byte
    TxData[5] = TxData[1] + TxData[2] + TxData[3] + TxData[4];  //check sum
    if (TxData[5] == 0x17)
        TxData[5] = 0x18;
    else
        TxData[5] = TxData[5];
    TxData[6] = 0x17;      //back code
    TxData[7] = 0x17;      //back code
}

void CTrimReader::SetGainMode(int gain)
{
    TxData[0] = 0xaa;      //preamble code
    TxData[1] = 0x01;      //command
    TxData[2] = 0x02;      //data length
    TxData[3] = 0x07;      //data type, date edit first byte
    TxData[4] = gain;      //real data, date edit second byte
    TxData[5] = TxData[1] + TxData[2] + TxData[3] + TxData[4];  //check sum
    if (TxData[5] == 0x17)
        TxData[5] = 0x18;
    else
        TxData[5] = TxData[5];
    TxData[6] = 0x17;      //back code
    TxData[7] = 0x17;      //back code
}

void CTrimReader::SetV15(uint8_t v15)
{
    TxData[0] = 0xaa;      //preamble code
    TxData[1] = 0x01;      //command
    TxData[2] = 0x02;      //data length
    TxData[3] = 0x05;      //data type, date edit first byte
    TxData[4] = v15;       //real data, date edit second byte

    TxData[5] = TxData[1] + TxData[2] + TxData[3] + TxData[4];  //check sum
    if (TxData[5] == 0x17)
        TxData[5] = 0x18;
    else
        TxData[5] = TxData[5];
    TxData[6] = 0x17;      //back code
    TxData[7] = 0x17;      //back code
}

void CTrimReader::Capture12()
{
    TxData[0] = 0xaa;      //preamble code
    TxData[1] = 0x02;      //command
    TxData[2] = 0x0C;      //data length
    TxData[3] = 0x02;      //data type, date edit first byte
    TxData[4] = 0xff;      //real data
    TxData[5] = 0x00;      //Reserved
    TxData[6] = 0x00;
    TxData[7] = 0x00;
    TxData[8] = 0x00;
    TxData[9] = 0x00;
    TxData[10] = 0x00;
    TxData[11] = 0x00;
    TxData[12] = 0x00;
    TxData[13] = 0x00;
    TxData[14] = 0x00;
    TxData[15] = TxData[1] + TxData[2] + TxData[3] + TxData[4] + TxData[5] + TxData[6] + TxData[7] + TxData[8] + TxData[9]
        + TxData[10] + TxData[11] + TxData[12] + TxData[13] + TxData[14];   //check sum
    if (TxData[15] == 0x17)
        TxData[15] = 0x18;
    else
        TxData[15] = TxData[15];
    TxData[16] = 0x17;     //back code
    TxData[17] = 0x17;     //back code
}

void CTrimReader::Capture12(uint8_t chan)
{
    if (chan < 1 || chan > 4)
        return;

    chan -= 1;

    TxData[0] = 0xaa;      //preamble code
    TxData[1] = 0x02;      //command
    TxData[2] = 0x0C;      //data length
    TxData[3] = (chan << 4) | 0x02;  //data type, date edit first byte
    TxData[4] = 0xff;      //real data
    TxData[5] = 0x00;      //Reserved
    TxData[6] = 0x00;
    TxData[7] = 0x00;
    TxData[8] = 0x00;
    TxData[9] = 0x00;
    TxData[10] = 0x00;
    TxData[11] = 0x00;
    TxData[12] = 0x00;
    TxData[13] = 0x00;
    TxData[14] = 0x00;
    TxData[15] = TxData[1] + TxData[2] + TxData[3] + TxData[4] + TxData[5] + TxData[6] + TxData[7] + TxData[8] + TxData[9]
        + TxData[10] + TxData[11] + TxData[12] + TxData[13] + TxData[14];   //check sum
    if (TxData[15] == 0x17)
        TxData[15] = 0x18;
    else
        TxData[15] = TxData[15];
    TxData[16] = 0x17;     //back code
    TxData[17] = 0x17;     //back code
}

void CTrimReader::Capture24()
{
    TxData[0] = 0xaa;      //preamble code
    TxData[1] = 0x02;      //command
    TxData[2] = 0x0C;      //data length
    TxData[3] = 0x08;      //data type, date edit first byte
    TxData[4] = 0xff;      //real data
    TxData[5] = 0x00;      //Reserved
    TxData[6] = 0x00;
    TxData[7] = 0x00;
    TxData[8] = 0x00;
    TxData[9] = 0x00;
    TxData[10] = 0x00;
    TxData[11] = 0x00;
    TxData[12] = 0x00;
    TxData[13] = 0x00;
    TxData[14] = 0x00;
    TxData[15] = TxData[1] + TxData[2] + TxData[3] + TxData[4] + TxData[5] + TxData[6] + TxData[7] + TxData[8] + TxData[9]
        + TxData[10] + TxData[11] + TxData[12] + TxData[13] + TxData[14];   //check sum
    if (TxData[15] == 0x17)
        TxData[15] = 0x18;
    else
        TxData[15] = TxData[15];
    TxData[16] = 0x17;     //back code
    TxData[17] = 0x17;     //back code
}

void CTrimReader::SetRangeTrim(uint8_t range)
{
    TxData[0] = 0xaa;      //preamble code
    TxData[1] = 0x01;      //command
    TxData[2] = 0x02;      //data length
    TxData[3] = 0x02;      //data type, date edit first byte
    TxData[4] = range;     //real data, date edit second byte
    //0x01 means send vedio data
    //0x00 means stop vedio data
    TxData[5] = TxData[1] + TxData[2] + TxData[3] + TxData[4];  //check sum
    if (TxData[5] == 0x17)
        TxData[5] = 0x18;
    else
        TxData[5] = TxData[5];
    TxData[6] = 0x17;      //back code
    TxData[7] = 0x17;      //back code
}

void CTrimReader::SetRampgen(uint8_t rampgen)
{
    TxData[0] = 0xaa;      //preamble code
    TxData[1] = 0x01;      //command
    TxData[2] = 0x02;      //data length
    TxData[3] = 0x01;      //data type, date edit first byte
    TxData[4] = rampgen;   //real data, date edit second byte
    //0x01 means send video data
    //0x00 means stop video data
    TxData[5] = TxData[1] + TxData[2] + TxData[3] + TxData[4];  //check sum
    if (TxData[5] == 0x17)
        TxData[5] = 0x18;
    else
        TxData[5] = TxData[5];
    TxData[6] = 0x17;      //back code
    TxData[7] = 0x17;      //back code
}

void CTrimReader::SetTXbin(uint8_t txbin)
{
    TxData[0] = 0xaa;      //preamble code
    TxData[1] = 0x01;      //command
    TxData[2] = 0x02;      //data length
    TxData[3] = 0x08;      //data type, date edit first byte
    TxData[4] = txbin;     //real data, date edit second byte
    //0x01 means send vedio data
    //0x00 means stop vedio data
    TxData[5] = TxData[1] + TxData[2] + TxData[3] + TxData[4];  //check sum
    if (TxData[5] == 0x17)
        TxData[5] = 0x18;
    else
        TxData[5] = TxData[5];
    TxData[6] = 0x17;      //back code
    TxData[7] = 0x17;      //back code
}

void CTrimReader::SetLEDConfig(bool IndvEn, bool Chan1, bool Chan2, bool Chan3, bool Chan4)
{
    TxData[0] = 0xaa;      //preamble code
    TxData[1] = 0x01;      //command
    TxData[2] = 0x02;      //data length
    TxData[3] = 0x23;      //data type, date edit first byte

    if (!IndvEn) {
        TxData[4] = Chan1 ? 1 : 0;  //real data, date edit second byte
    }
    else {
        TxData[4] = 0x80;
        if (Chan1)
            TxData[4] |= 1;
        if (Chan2)
            TxData[4] |= 2;
        if (Chan3)
            TxData[4] |= 4;
        if (Chan4)
            TxData[4] |= 8;
    }

    TxData[5] = TxData[1] + TxData[2] + TxData[3] + TxData[4];  //check sum

    if (TxData[5] == 0x17)
        TxData[5] = 0x18;
    else
        TxData[5] = TxData[5];

    TxData[6] = 0x17;      //back code
    TxData[7] = 0x17;      //back code
}

void CTrimReader::SetIntTime(float int_t)
{
    unsigned char* hData = (unsigned char*)&int_t;  // Get bytes from float

    uint8_t TrimBuf[8];

    TrimBuf[0] = hData[0];  //buffer
    TrimBuf[1] = hData[1];
    TrimBuf[2] = hData[2];
    TrimBuf[3] = hData[3];

    TxData[0] = 0xaa;       //preamble code
    TxData[1] = 0x01;       //command
    TxData[2] = 0x05;       //data length
    TxData[3] = 0x20;       //data type, date edit first byte
    TxData[4] = TrimBuf[0]; //real data, date edit second byte
    TxData[5] = TrimBuf[1];
    TxData[6] = TrimBuf[2];
    TxData[7] = TrimBuf[3];
    //0x01 means send vedio data
    //0x00 means stop vedio data
    TxData[8] = TxData[1] + TxData[2] + TxData[3] + TxData[4] + TxData[5] + TxData[6] + TxData[7]; //check sum
    if (TxData[8] == 0x17)
        TxData[8] = 0x18;
    else
        TxData[5] = TxData[5];
    TxData[9] = 0x17;       //back code
    TxData[10] = 0x17;      //back code
}

void CTrimReader::SelSensor(uint8_t i)
{
    // Check sensor channel bounds
    if (i < 1 || i > 4) return;

    TxData[0] = 0xaa;       //preamble code
    TxData[1] = 0x01;       //command
    TxData[2] = 0x03;       //data length
    TxData[3] = 0x26;       //data type
    TxData[4] = i - 1;      //real data
    TxData[5] = 0x00;
    TxData[6] = TxData[1] + TxData[2] + TxData[3] + TxData[4] + TxData[5];  //check sum
    if (TxData[6] == 0x17)
        TxData[6] = 0x18;
    else
        TxData[6] = TxData[6];
    TxData[7] = 0x17;       //back code
    TxData[8] = 0x17;       //back code
}

// Page display types
#define dppage12 0x02       // display one page with 12 pixel
#define dppage24 0x08       // display one page with 24 pixel

int CTrimReader::ProcessRowData(int (*adc_data)[24], int gain_mode)
{
    int result;
    int flag, ncol = 12;
    int FrameSize = 0;

    // Get the data type from the correct location
    uint8_t type = RxData[4];    // data type

    // Determine frame size based on data type
    switch (type & 0x0F)  // Use only the lower 4 bits for frame type
    {
    case dppage12:        // 12x12 frame
        ncol = 12;
        FrameSize = 0;
        break;

    case dppage24:        // 24x24 frame
        ncol = 24;
        FrameSize = 1;
        break;

    default:
        printf("Warning: Unknown frame type: %02x\n", type);
        break;
    }

    // Process each column for this row
    unsigned int rn = RxData[5]; // row number

    for (int i = 0; i < ncol; i++)
    {
        // Windows code uses data stride of 2 with low byte first, high byte second
        uint8_t low_byte = RxData[i * 2 + 6];
        uint8_t high_byte = RxData[i * 2 + 7];

        // Process the pixel data
        result = ADCCorrectioni(i, high_byte, low_byte, ncol, chan_num, gain_mode, &flag);

        // Store the result in the data array
        adc_data[rn][i] = result;

        // Ensure no negative values
        if (adc_data[rn][i] < 0) adc_data[rn][i] = 0;
    }

    // Check for end signal
    if (rn == 0x0b) {
        Continue_Flag = false;
    }

    return FrameSize;
}

uint8_t CTrimReader::TrimBuff2Byte()
{
    uint8_t r = trim_buff[tbuff_rptr++];
    return r;
}

void CTrimReader::CopyEepromBuffAndRestore()
{
    // Copy first page and restore header
    for (int j = 0; j < EPKT_SZ; j++) {  // parity not copied
        trim_buff[j] = EepromBuff[0][j]; // copy first page
    }

    RestoreFromTrimBuff();

    // Copy remaining pages
    for (int i = 1; i < num_pages; i++) {
        for (int j = 0; j < EPKT_SZ; j++) {  // parity not copied
            trim_buff[i * EPKT_SZ + j] = EepromBuff[i][j];
        }
    }
}

void CTrimReader::RestoreFromTrimBuff()
{
    // restore trimbuff header first
    tbuff_rptr = 0;  // initialize read pointer

    id = TrimBuff2Byte();

    if (id != 0xa5) {
        serial_number1 = TrimBuff2Byte();
        serial_number2 = TrimBuff2Byte();
        num_channels = TrimBuff2Byte();
        num_wells = TrimBuff2Byte();
        num_pages = TrimBuff2Byte();
    }
    else {
        version = TrimBuff2Byte();
        num_pages = TrimBuff2Byte();

        id_str.clear();
        for (int i = 0; i < 32; i++) {
            id_str.push_back(TrimBuff2Byte());
        }

        serial_number1 = TrimBuff2Byte();
        serial_number2 = TrimBuff2Byte();

        num_channels = TrimBuff2Byte();
        num_wells = TrimBuff2Byte();

        well_format = TrimBuff2Byte();
        channel_format = TrimBuff2Byte();
    }
}

extern bool ee_continue;

void CTrimReader::OnEEPROMRead()
{
    // EEPROM data, check parity here too.
    uint8_t eeprom_parity = 0;
    int index = RxData[7];    // For command type 2d EEPROM read command
    int npages = RxData[6];
    bool parity_ok = true;

    for (int i = 0; i < EPKT_SZ + 1; i++) {  // +1 for parity
        EepromBuff[index][i] = RxData[8 + i];

        if (i < EPKT_SZ) {
            eeprom_parity += RxData[8 + i];
        }
        else {
            if (eeprom_parity != RxData[8 + i]) {
                // Log parity error
                parity_ok = false;
            }
        }
    }

    // Update continue flag for reading next page
    ee_continue = (index < npages - 1);
}

void CTrimReader::EEPROMRead()
{
    TxData[0] = 0xaa;          //preamble code
    TxData[1] = 0x04;          //command
    TxData[2] = 0x02;          //data length
    TxData[3] = 0x2d;          //data type
    TxData[4] = (uint8_t)0x0;  //real data
    TxData[5] = TxData[1] + TxData[2] + TxData[3] + TxData[4];  //check sum
    if (TxData[5] == 0x17)
        TxData[5] = 0x18;
    else
        TxData[5] = TxData[5];
    TxData[6] = 0x17;          //back code
    TxData[7] = 0x17;          //back code
}

void CTrimReader::ReadTrimData()
{
    CopyEepromBuffAndRestore();  // Compare with g_DPReader

    int nchannels = num_channels;
    int npages = num_pages;

    NumNode = nchannels;

    for (int i = 0; i < nchannels; i++) {
        CopyEepromBuff(i, npages + i * NUM_EPKT);
        RestoreTrimBuff(i);
        Node[i].version = 3;  // Use integer version KB matrix and FPN values
    }
}

// EEProm buffer related stuff

void CTrimReader::Convert2Int(int c)
{
    int intmax = 32767;
    int intmax256 = 128;

    int i;
    curNode = &Node[c];

    for (i = 0; i < 12; i++) {
        curNode->kbi[i][0] = (int)round(curNode->kb[i][0] * (double)intmax);
        curNode->kbi[i][1] = (int)round(curNode->kb[i][1] * (double)intmax256);
        curNode->kbi[i][2] = (int)round(curNode->kb[i][2] * (double)intmax);
        curNode->kbi[i][3] = (int)round(curNode->kb[i][3] * (double)intmax256);
        curNode->kbi[i][4] = (int)round(curNode->kb[i][4] * (double)intmax256);
        curNode->kbi[i][5] = (int)round(curNode->kb[i][5] * (double)intmax256);
    }

    for (i = 0; i < 12; i++) {
        curNode->fpni[0][i] = (int)round(curNode->fpn[0][i]);
        curNode->fpni[1][i] = (int)round(curNode->fpn[1][i]);
    }
}

int CTrimReader::Add2TrimBuff(int i, int val)
{
    int k = Node[i].tbuff_size;
    if (k >= MAX_TRIMBUFF - 1) return -1;

    Node[i].trim_buff[k] = val >> 8;       // High byte
    Node[i].trim_buff[k + 1] = val & 0xFF; // Low byte
    Node[i].tbuff_size = k + 2;

    return k + 2;
}

int CTrimReader::Add2TrimBuff(int i, uint8_t val)
{
    int k = Node[i].tbuff_size;
    if (k >= MAX_TRIMBUFF) return -1;

    Node[i].trim_buff[k] = val;
    Node[i].tbuff_size = k + 1;

    return k + 1;
}

int CTrimReader::WriteTrimBuff(int k)
{
    int i, j;
    Node[k].tbuff_size = 0;  // initialize write pointer

    // Add identification bytes
    Add2TrimBuff(k, (uint8_t)Node[k].name[0]);
    Add2TrimBuff(k, (uint8_t)Node[k].name[1]);
    Add2TrimBuff(k, (uint8_t)Node[k].name[2]);

    // Add calibration data
    for (i = 0; i < TRIM_IMAGER_SIZE; i++) {
        for (j = 0; j < 6; j++) {
            Add2TrimBuff(k, (int)Node[k].kbi[i][j]);  // kb
        }
    }

    // Add FPN data
    for (i = 0; i < TRIM_IMAGER_SIZE; i++) {
        Add2TrimBuff(k, (int)(Node[k].fpni[0][i]));
        Add2TrimBuff(k, (int)(Node[k].fpni[1][i]));
    }

    // Add hardware parameters
    Add2TrimBuff(k, (uint8_t)(Node[k].rampgen));
    Add2TrimBuff(k, (uint8_t)(Node[k].range));
    Add2TrimBuff(k, (uint8_t)(Node[k].auto_v20[0]));
    Add2TrimBuff(k, (uint8_t)(Node[k].auto_v20[1]));
    Add2TrimBuff(k, (uint8_t)(Node[k].auto_v15));

    // Add temperature calibration
    Add2TrimBuff(k, (int)round(29.5 * 128));  // tempcal1

    float tcal2 = 0;  // Default second calibration point
    int r = Add2TrimBuff(k, (int)round(tcal2 * 128));

    return r;
}

int CTrimReader::TrimBuff2Int(int i)
{
    int16_t r;  // Use int16_t to properly handle negative values
    int k = Node[i].tbuff_rptr;

    r = (Node[i].trim_buff[k] << 8) | (Node[i].trim_buff[k + 1]);
    Node[i].tbuff_rptr += 2;

    return (int)r;
}

uint8_t CTrimReader::TrimBuff2Byte(int i)
{
    uint8_t r = Node[i].trim_buff[Node[i].tbuff_rptr++];
    return r;
}

void CTrimReader::RestoreTrimBuff(int k)
{
    int i, j;
    Node[k].tbuff_rptr = 0;  // initialize read pointer

    uint8_t b0, b1, b2;

    b0 = TrimBuff2Byte(k);
    b1 = TrimBuff2Byte(k);
    b2 = TrimBuff2Byte(k);

    int sn = b1 << 8 | b0;

    // Read calibration data
    for (i = 0; i < TRIM_IMAGER_SIZE; i++) {
        for (j = 0; j < 6; j++) {
            Node[k].kbi[i][j] = TrimBuff2Int(k);
        }
    }

    // Read FPN data
    for (i = 0; i < TRIM_IMAGER_SIZE; i++) {
        Node[k].fpni[0][i] = TrimBuff2Int(k);
        Node[k].fpni[1][i] = TrimBuff2Int(k);
    }

    // Read hardware parameters
    Node[k].rampgen = TrimBuff2Byte(k);
    Node[k].range = TrimBuff2Byte(k);
    Node[k].auto_v20[0] = TrimBuff2Byte(k);
    Node[k].auto_v20[1] = TrimBuff2Byte(k);
    Node[k].auto_v15 = TrimBuff2Byte(k);

    // Read temperature calibration
    Node[k].tempcal[0] = (double)TrimBuff2Int(k) / (double)128;
    Node[k].tempcal[1] = (double)TrimBuff2Int(k) / (double)128;
}

void CTrimReader::CopyEepromBuff(int k, int index_start)
{
    for (int i = 0; i < NUM_EPKT; i++) {
        for (int j = 0; j < EPKT_SZ; j++) {  // parity not copied
            Node[k].trim_buff[i * EPKT_SZ + j] = EepromBuff[i + index_start][j];
        }
    }
}