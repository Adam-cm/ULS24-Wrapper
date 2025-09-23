// Copyright 2014-2017, Anitoa Systems, LLC
// All rights reserved

#pragma once

#include <string>
#include <fstream>
#include <cstdint>

#define TRIM_IMAGER_SIZE 12
#define MAX_TRIMBUFF 256

#define EPKT_SZ  52
#define NUM_EPKT 4

class CTrimNode {
public:
    double kb[TRIM_IMAGER_SIZE][6];
    double fpn[2][TRIM_IMAGER_SIZE];

    unsigned int rampgen;
    unsigned int range;
    unsigned int auto_v20[2];
    unsigned int auto_v15;
    unsigned int version;

    double tempcal[TRIM_IMAGER_SIZE];

    int kbi[TRIM_IMAGER_SIZE][6];
    int fpni[2][TRIM_IMAGER_SIZE];

    std::string name; // Fixed: previously missing type

    uint8_t trim_buff[MAX_TRIMBUFF];
    int tbuff_size;
    int tbuff_rptr;

    CTrimNode();

private:
    void Initialize();
};

#define TRIM_MAX_NODE 4
#define TRIM_MAX_WORD 640

class CTrimReader {
protected:
    std::string WordBuf[TRIM_MAX_WORD];
    int WordIndex;
    int MaxWord;
    std::string CurWord;

    uint8_t trim_buff[1024];
    int tbuff_size;
    int tbuff_rptr;

    uint8_t version;
    uint8_t id;
    std::string id_str;
    uint8_t serial_number1, serial_number2;
    uint8_t num_wells, num_channels, well_format, channel_format;
    uint8_t num_pages;

public:
    CTrimNode Node[TRIM_MAX_NODE];
    CTrimNode* curNode;
    int NumNode;

    CTrimReader();
    ~CTrimReader();

    int Load(const std::string& filename); // Changed from TCHAR* to std::string
    void Parse();
    void ParseNode();

    int GetNumNode() const { return NumNode; }

    int ADCCorrection(int NumData, uint8_t HighByte, uint8_t LowByte, int pixelNum, int PCRNum, int gain_mode, int* flag);
    int ADCCorrectioni(int NumData, uint8_t HighByte, uint8_t LowByte, int pixelNum, int PCRNum, int gain_mode, int* flag);

    void SetV20(uint8_t v20);
    void SetGainMode(int gain);
    void SetV15(uint8_t v15);
    void Capture12();
    void Capture12(uint8_t);
    void Capture24();
    int ProcessRowData(int (*adc_data)[24], int gain_mode);

    void SetRangeTrim(uint8_t range);
    void SetRampgen(uint8_t rampgen);
    void SetTXbin(uint8_t txbin);
    void SetIntTime(float);
    void SelSensor(uint8_t i);
    void SetIntTime(float kfl, uint8_t ch);
    void SetLEDConfig(bool IndvEn, bool Chan1, bool Chan2, bool Chan3, bool Chan4);

    void EEPROMRead();
    void OnEEPROMRead();
    void ReadTrimData();

    uint8_t TrimBuff2Byte();
    void CopyEepromBuffAndRestore();
    void RestoreFromTrimBuff();

    int Add2TrimBuff(int i, int);
    int Add2TrimBuff(int i, uint8_t);
    int WriteTrimBuff(int i);
    int TrimBuff2Int(int i);
    uint8_t TrimBuff2Byte(int i);
    void RestoreTrimBuff(int k);
    void CopyEepromBuff(int k, int index_start);

    void Convert2Int(int c);

protected:
    bool fileLoaded;

    void ParseMatrix();
    void ParseArray(int);
    void ParseValue(int);

private:
    int GetWord();
    int Match(const std::string&);
};